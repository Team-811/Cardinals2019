/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.MecanumDrive;
import frc.robot.lib.MotionProfiling;
import frc.robot.lib.Output;
import frc.robot.lib.PIDController;
import frc.robot.lib.TalonChecker;
import frc.robot.lib.UnitConverter;
import frc.robot.lib.vision.AimingParameters;
import frc.robot.RobotState;
import jaci.pathfinder.Waypoint;
import frc.robot.commands.Drivetrain.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  This subsystem deals with the 
 * mecanum drivetrain.  The drivetrain includes 4 motors with encoders on each, an encoder on a dead wheel to track strafing, and 
 * a gyro to measure the angle of the robot.  For controlling the subsystem, there are methods that do field oriented driving with
 * gyro correction for joystick control, PID loops for the vision, and Motion Profiling for auto driving.
 */

public class Drivetrain extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Drivetrain instance = new Drivetrain();

  public static Drivetrain getInstance() {
    return instance;
  }

  private TalonSRX topLeftMotor;
  private TalonSRX topRightMotor;
  private TalonSRX bottomLeftMotor;
  private TalonSRX bottomRightMotor;

  private AHRS gyro;

  private MecanumDrive drivetrain;
  private MotionProfiling motionProfile;

  private RobotState robot_state_;

  private PIDController pidDistance;
  private PIDController pidStrafe;
  private PIDController pidRotation;
  private double distanceOffset;
  private double strafeOffset;
  private double rotationOffset;

  //private Encoder strafeEncoder;
  
  private double kPForward = 1.2;
  private double kIForward = 0;
  private double kDForward = 0.05;
  private double kFForward = 0;

  private double kPRotation = 0.021;
  private double kIRotation = 0;
  private double kDRotation = 0.000006;
  private double kFRotation;

  private double kPStrafe;
  private double kIStrafe;
  private double kDStrafe;
  private double kFStrafe;

  private double kPVelocity;
  private double kIVelocity;
  private double kDVelocity;
  private double kFVelocity;

  private int kTimeoutMs;

  public Drivetrain()
  {
      topLeftMotor = new TalonSRX(RobotMap.DRIVE_TOP_LEFT_MOTOR);
      topRightMotor = new TalonSRX(RobotMap.DRIVE_TOP_RIGHT_MOTOR);
      bottomLeftMotor = new TalonSRX(RobotMap.DRIVE_BOTTOM_LEFT_MOTOR);
      bottomRightMotor = new TalonSRX(RobotMap.DRIVE_BOTTOM_RIGHT_MOTOR);

      //strafeEncoder = new Encoder(RobotMap.DRIVE_STRAFE_ENCODER_ACHANNEL, RobotMap.DRIVE_STRAFE_ENCODER_BCHANNEL);

      robot_state_ = RobotState.getInstance();

      configureTalons();

      gyro = new AHRS(SerialPort.Port.kMXP);
      gyro.reset();
      invertGyro(false);

      zeroEncoders();

      drivetrain = new MecanumDrive();
      motionProfile = new MotionProfiling(Constants.maxVelocity, Constants.maxAcceleration, Constants.maxJerk, Constants.wheelbase);
      configurePID();
      

      drivetrain.invertForwardBackward(false);
      drivetrain.invertStrafing(true);
      drivetrain.invertRotation(false);
  }


  public enum DriveMode{
    Arcade(),
    FieldOriented();
  }

  private DriveMode mode = DriveMode.Arcade;

  public DriveMode getMode() {
    return mode;
  }

  public void setDriveMode(DriveMode newMode) {
    this.mode = newMode;
  }

  public void toggleDriveMode()
  {
    if(mode == DriveMode.Arcade)
      mode = DriveMode.FieldOriented;
    else
      mode = DriveMode.Arcade;
  }

  private double SpeedScale = 1;


  public void DriveWithJoy(double forward, double rotation, double strafe)
  {
      Output driveOutput;
      double correction;

      if(rotation < 0.2 && rotation > -0.2)
          //when not rotating, compare your current gyro pos to the last time you were rotating to get error
          correction = gyroCorrection();
        else
          correction = 0;


      if(mode == DriveMode.Arcade)
      {
        if(gyro.isConnected())
          driveOutput = drivetrain.arcadeMecanumDrive(forward * SpeedScale, (rotation - correction) * SpeedScale, strafe * SpeedScale);
        else
          driveOutput = drivetrain.arcadeMecanumDrive(forward * SpeedScale, rotation * SpeedScale, strafe * SpeedScale);
      }
      else
      {
        if(gyro.isConnected())
          driveOutput = drivetrain.fieldOrientedDrive(forward * SpeedScale, (rotation - correction) * SpeedScale, strafe * SpeedScale, getGyroAngle());
        else
          driveOutput = drivetrain.arcadeMecanumDrive(forward * SpeedScale, rotation * SpeedScale, strafe * SpeedScale);
      }

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());

      
      prevAngle = getGyroAngle(); //Stores previous angle

  }

  public void slowMode(boolean isSlow)
  {
    if(isSlow)
        SpeedScale = 0.5;
    else
        SpeedScale = 1;

  }

  public void stopDrivetrain()
  {
    topLeftMotor.set(ControlMode.PercentOutput, 0);
    topRightMotor.set(ControlMode.PercentOutput, 0);
    bottomLeftMotor.set(ControlMode.PercentOutput, 0);
    bottomRightMotor.set(ControlMode.PercentOutput, 0);
  }


  //Motion Profiling

  public void loadTrajectory(Waypoint[] path, boolean reverse)
  {
      motionProfile.loadTrajectory(path, reverse);
  }

  public void followTrajectory(boolean reverse)
  {

    Output driveOutput = motionProfile.getNextDriveSignal(reverse, getLeftEncoder(), getRightEncoder(), gyro.getAngle(), false);

    double velocityLeft = UnitConverter.metersPerSecondToTalonUnits(driveOutput.getLeftValue(), Constants.wheelDiameter, Constants.ticksPerRotation);
    double velocityRight = UnitConverter.metersPerSecondToTalonUnits(driveOutput.getRightValue(), Constants.wheelDiameter, Constants.ticksPerRotation);

    setVelocity(velocityLeft, velocityRight);
  }

  public boolean isPathFinished()
  {
      return motionProfile.isFinished();
  }

  public double percentagePathFinished()
  {
      return motionProfile.percentageDone();
  }

  public boolean isPathPercentDone(double percentage)
  {
      return percentage <= percentagePathFinished();
  }


  public void loadTrajectoryPathfinder(Waypoint[] path, boolean reverse)
  {
      double leftEncoder = (topLeftMotor.getSelectedSensorPosition() + bottomLeftMotor.getSelectedSensorVelocity()) / 2;
      double rightEncoder = (topRightMotor.getSelectedSensorPosition() + bottomRightMotor.getSelectedSensorVelocity()) / 2;
      motionProfile.loadTrajectoryPathfinder(path, reverse, (int)leftEncoder, (int)rightEncoder, getGyroAngle());
  }

  public void followTrajectoryPathfinder(boolean reverse)
  {
      double leftEncoder = (topLeftMotor.getSelectedSensorPosition() + bottomLeftMotor.getSelectedSensorVelocity()) / 2;
      double rightEncoder = (topRightMotor.getSelectedSensorPosition() + bottomRightMotor.getSelectedSensorVelocity()) / 2;

    Output driveOutput = motionProfile.followTrajectoryPathfinder(reverse, (int)leftEncoder, (int)rightEncoder, gyro.getAngle());

    topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
    topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
    bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
    bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());
  }

  public boolean isPathFinishedPathfinder()
  {
      return motionProfile.isPathfinderPathDone();
  }



  //Velocity Control

  public void setVelocity(double velocityLeft, double velocityRight)
  {
    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Follower, topLeftMotor.getDeviceID());
    bottomLeftMotor.set(ControlMode.Follower, topRightMotor.getDeviceID());

  }


  //PID Control

  public void PIDControl()
  {
      
      //Pick any encoder to track istance
      double distance = pidDistance.updatePID(getLeftEncoder(), distanceOffset);
      //Strafe encoder to track strafing
      double strafe = pidStrafe.updatePID(getStrafeEncoder(), strafeOffset);
      //Gyro to track rotation
      double rotation = pidRotation.updatePID(getGyroAngle(), rotationOffset);

      Output driveOutput = drivetrain.arcadeMecanumDrive(0, 0, -rotation);

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());
  } 

  public void setPIDGoals(double goalDistance, double goalStrafe, double goalAngleInDegrees)
  {
      distanceOffset = getForwardEncoder() + goalDistance;
      rotationOffset = getGyroAngle() + goalAngleInDegrees;
      strafeOffset = getStrafeEncoder() + goalStrafe;

  }

  public boolean pidIsDone()
  {
    return pidDistance.isOnTarget() && pidStrafe.isOnTarget() && pidRotation.isOnTarget();
  }

  public void setUpVisionPID()
  {
      AimingParameters target = robot_state_.getAimingParameters();

      if(target != null)
        setPIDGoals(target.getY(), target.getX(), 0); //Assumes drivers rotation is good enough
      else
        System.out.println("No targets to track");

  }




  //Gyro Correction

  private double gyroCorrectRate = 0.1;

  private double prevAngle;

  public double gyroCorrection()
  {
      return (getGyroAngle() - prevAngle) * gyroCorrectRate;
  }


  //Encoder Distances

  public double getTopLeftEncoder()
  {
    return UnitConverter.ticksToMeters(topLeftMotor.getSelectedSensorPosition(), 1378, Constants.wheelDiameter);
  }

  public double getTopRightEncoder()
  {
    return UnitConverter.ticksToMeters(topRightMotor.getSelectedSensorPosition(), 1373, Constants.wheelDiameter);
  }

  public double getBottomLeftEncoder()
  {
    return UnitConverter.ticksToMeters(bottomLeftMotor.getSelectedSensorPosition(), 1024, Constants.wheelDiameter);
  }

  public double getBottomRightEncoder()
  {
    return UnitConverter.ticksToMeters(bottomRightMotor.getSelectedSensorPosition(), 1024, Constants.wheelDiameter);
  }

  public double getLeftEncoder()
  {
    return (getTopLeftEncoder() + getBottomLeftEncoder()) / 2;
  }

  public double getLeftStrafe()
  {
    return (getTopLeftEncoder() - getBottomLeftEncoder()) * Constants.strafeTrackScrubFactor / 2;
  }

  public double getRightEncoder()
  {
    return (getTopRightEncoder() + getBottomRightEncoder()) / 2;
  }

  public double getRightStrafe()
  {
    return -(getTopRightEncoder() - getBottomRightEncoder()) * Constants.strafeTrackScrubFactor / 2;
  }

  public double getForwardEncoder()
  {
    return (getLeftEncoder() + getRightEncoder()) / 2;
  }
  public double getStrafeEncoder()
  {
    return (getLeftStrafe() + getRightStrafe()) / 2;
  }

  public void zeroEncoders()
  {
    topLeftMotor.setSelectedSensorPosition(0);
    topRightMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);
  }




  //Encoder Velocities

  public double getTopLeftVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(topLeftMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getTopRightVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(topRightMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getBottomLeftVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(bottomLeftMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getBottomRightVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(bottomRightMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getLeftVelocity()
  {
    return (getTopLeftVelocity() + getBottomLeftVelocity()) / 2;
  }

  public double getLeftStrafeVelocity()
  {
    return (getTopLeftVelocity() - getBottomLeftVelocity()) * Constants.strafeTrackScrubFactor / 2;
  }

  public double getRightVelocity()
  {
    return (getTopRightVelocity() + getBottomRightVelocity()) / 2;
  }

  public double getRightStrafeVelocity()
  {
    return -(getTopRightVelocity() - getBottomRightVelocity()) * Constants.strafeTrackScrubFactor / 2;
  }

  public double getForwardVelocity()
  {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }

  public double getStrafeVelocity()
  {
    //Divided by 10 because getRate time measurement is in seconds and not 100ms
    return (getLeftStrafeVelocity() + getRightStrafeVelocity()) / 2;
  }






  //Gyro measurements

  private int gyroInversion = 1;

  public double getGyroAngle()
  {
      return gyroInversion *gyro.getAngle();
  }

  public double getAngularVelocity()
  {
      return gyroInversion * gyro.getRate();
  }

  public void invertGyro(boolean inverted)
  {
      if(inverted)
        gyroInversion = -1;
      else
        gyroInversion = 1;
  }

  public void zeroGyro()
  {
      gyro.zeroYaw();
  }

  public AHRS getGyro()
  {
      return gyro;
  }




  private void configureTalons()
  {
    topLeftMotor.setInverted(false);
    topRightMotor.setInverted(true);
    bottomLeftMotor.setInverted(false);
    bottomRightMotor.setInverted(true);

    topLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    topRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    bottomLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    bottomRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    topLeftMotor.setSensorPhase(false);
    topRightMotor.setSensorPhase(true);
    bottomLeftMotor.setSensorPhase(true);
    bottomRightMotor.setSensorPhase(true);

    bottomLeftMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);

    bottomLeftMotor.configNominalOutputForward(0);
    bottomLeftMotor.configNominalOutputReverse(0);
    bottomLeftMotor.configPeakOutputForward(1);
    bottomLeftMotor.configPeakOutputReverse(-1);

    topLeftMotor.config_kP(0, kPVelocity);
    topLeftMotor.config_kI(0, kIVelocity);
    topLeftMotor.config_kD(0, kDVelocity); 
    topLeftMotor.config_kF(0, kFVelocity);

    topRightMotor.config_kP(0, kPVelocity);
    topRightMotor.config_kI(0, kIVelocity);
    topRightMotor.config_kD(0, kDVelocity);
    topRightMotor.config_kF(0, kFVelocity);

  }

  private void configurePID()
  {
      pidDistance = new PIDController(kPForward, kIForward, kDForward);
      pidDistance.setThreshold(0.02);
      pidDistance.setOutputConstraints(0.8, -0.8);
      pidStrafe = new PIDController(kPStrafe, kIStrafe, kDStrafe);
      pidStrafe.setThreshold(0.03);
      pidStrafe.setOutputConstraints(1, -1);
      pidRotation  = new PIDController(kPRotation, kIRotation, kDRotation);
      pidRotation.setThreshold(5);
      pidRotation.setOutputConstraints(0.8, -0.8);
  }

  


  @Override
  public void outputSmartdashboard() 
  {
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
      SmartDashboard.putNumber("Top Left Encoder", getTopLeftEncoder());
      SmartDashboard.putNumber("Top Right Encoder", getTopRightEncoder());
      SmartDashboard.putNumber("Bottom Left Encoder", getBottomLeftEncoder());
      SmartDashboard.putNumber("Bottom Right Encoder", getBottomRightEncoder());
      SmartDashboard.putNumber("Strafe", getStrafeEncoder());
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  }

  @Override
  public void zeroSensors() 
  {
    zeroEncoders();
    zeroGyro();
  }

  @Override
  public void resetSubsystem() 
  {
    stopDrivetrain();
    zeroSensors();
    configureTalons();
  }

  @Override
  public void testSubsystem() {
    boolean sucess = true;

    Timer delay = new Timer();
    System.out.println("///////////////////////////////////////////////////");
    System.out.println("***************Beginning Drivetrain Test***************");
    Timer.delay(0.2);


    //Test top left motor
    System.out.println("Testing Top Left Motor and Encoder");
    Timer.delay(0.5);
    TalonChecker checker = new TalonChecker("Top Left Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(5, 75); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Top Left Motor or Encoder***************");
        return;
    }

    //Test top right motor
    System.out.println("Testing Top Right Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Top Right Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(2, 75); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Top Right Motor or Encoder***************");
        return;
    }

    //Test bottom left motor
    System.out.println("Testing Bottom Left Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Bottom Left Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(2, 75); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Bottom Left Motor or Encoder***************");
        return;
    }

    //Test bottom right motor
    System.out.println("Testing Bottom Right Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Bottom Right Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(2, 75); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Bottom Right Motor or Encoder***************");
        return;
    }

    System.out.println("Testing Gyro");
    Timer.delay(0.5);
    sucess = gyro.isConnected();
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Gyro Not Connected***************");
        return;
    }


    if(sucess)
        System.out.println("***************Everything in Drivetrain is working***************");
    else
        System.out.println("***************Error in Drivetrain***************");
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
