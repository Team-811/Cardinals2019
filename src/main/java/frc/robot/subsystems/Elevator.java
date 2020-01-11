/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.lib.TalonChecker;
import frc.robot.commands.Elevator.*;
import frc.robot.lib.UnitConverter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  This subsystem deals with the
 * linear lift part of the robot.  The linear lift contains one motor for lifting, an encoder to track the position, and a 
 * limit switch at the bottom to stop the elevator and zero the encoder.  To control the elevator, the motors use motion magic
 * to do on the fly motion profiling.  The joystick control will just change the desired position of the elevator and the 
 * motion magic will move the evelvator to that postion.  There is also direct control which makes the motor spin however fast the joystick
 * is pushed.  Since the encoder for the elevator wasnt working this year only direct control was used.
 */

public class Elevator extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Elevator instance = new Elevator();

  public static Elevator getInstance() {
      return instance;
  }


  public static enum LiftState {
    GoingUp,
    GoingDown,
    Stationary,
    BottomedOut,
    ToppedOut,
}

public LiftState getState() {
    return state;
}

private void setState(LiftState newState) {
    this.state = newState;
}

private LiftState state = LiftState.Stationary;

private TalonSRX elevatorLeader = new TalonSRX(RobotMap.ELEVATOR_MAIN);
private DigitalInput bottomLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_BOTTOM_LIMIT_SWITCH);

private double kP = 0.5; // .3
private double kI = 0.0;
private double kD = 4.0; // 4.0
private double kF = 0.1165 * 2; // 0.1165 * 2

//max velocity of elevator: 0.888 m/s
//max motor RPM: 4500
//16:1 geared down
//
private final int CRUISE_VELOCITY = 40;
private final int CRUISE_ACCELERATION = 30;
private final int CRUISE_VELOCITY_DOWN = (int) (CRUISE_VELOCITY * 0.6); // 1024
private final int CRUISE_ACCELERATION_DOWN = (int) (CRUISE_ACCELERATION * 0.6); // 1024

private final double joyRate = 3;

public enum Positions {
    //TODO
    Intake(0),
    Level2Hatch(50000),
    Level3Hatch(22000),
    Level1Cargo(70000),
    Level2Cargo(75001),
    Level3Cargo(0),
    Top(720);
    private double position;

    Positions(double encPos) {
        this.position = encPos;
    }

    public double getPosition() {
        return this.position;
    }
}

private double position = Positions.Intake.position;

public double getDesiredPosition() {
    return position;
}

public void setPosition(double newPos) {
    this.position = newPos;
}

public void setPositionWithJoy(double joy) {
    setPosition(getDesiredPosition() + joyRate * 10);
}

private final int MOTION_MAGIC_TOLERANCE = 150;
private static final double ELEVATOR_HI_POW = 1.0;
private static final double ELEVATOR_LOW_POW = -ELEVATOR_HI_POW;

public Elevator() {
    //configure motors
    elevatorLeader.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    elevatorLeader.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    elevatorLeader.configNominalOutputForward(0.0, 0);
    elevatorLeader.configNominalOutputReverse(0.0, 0);

    //Encoder
    elevatorLeader.setSensorPhase(false);
    elevatorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    zeroSensors();

    //Neutral mode
    elevatorLeader.setNeutralMode(NeutralMode.Brake);

    elevatorLeader.setInverted(false);

    configPIDF(kP, kI, kD, kF);
    configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);

}

public double getEncoderPosition() {
    return elevatorLeader.getSelectedSensorPosition();
}

public double getEncoderVeloctiy()
{
    return elevatorLeader.getSelectedSensorVelocity();
}

public boolean getBottomLimit()
{
    return !bottomLimitSwitch.get();
}


//Motion Magic Methods

public void runMotionMagic() { 
    //If the desired position is higher than the current, then the elevator must go up
    if (getEncoderPosition() > position) {
        setState(LiftState.GoingUp);
        configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);
    
    //If the desired position is lower than the current, then the elevator must going down
    } else if (getEncoderPosition() < position) {          
        setState(LiftState.GoingDown);
        configMotionMagic(CRUISE_VELOCITY_DOWN, CRUISE_ACCELERATION_DOWN);
    }
    //If the elevator is at the desired position, the elevator is stationary
    if (Math.abs(getDesiredPosition() - getEncoderPosition()) <= MOTION_MAGIC_TOLERANCE) {
        state = LiftState.Stationary;
    }

    //check if topped out or zeroed out, else run motion magic
    if(!isToppedOut() && !isZeroedOut())
        elevatorLeader.set(ControlMode.MotionMagic, position);
    else
        stopElevator();
}

public void checkMotionMagicTermination() {
    //If at desired position, then the elevator is considered stationary
    if (Math.abs(getDesiredPosition() - getEncoderPosition()) <= MOTION_MAGIC_TOLERANCE) {
        state = LiftState.Stationary;
    }
    //If elevator is at limits, then stop the elevator
    
}



//Direct Control

public void directJoyControl(double joystick)
{
    
    if(joystick >= 0.1)
    {
        setState(LiftState.GoingUp);
    }
    else if(joystick <= -0.1)
    {
        setState(LiftState.GoingDown);
    }
    else
    {
        setState(LiftState.Stationary);
    }
    
    if(!isZeroedOut())
        elevatorLeader.set(ControlMode.PercentOutput, joystick);
    else
        stopElevator();
}


public void stopElevator() {
    elevatorLeader.set(ControlMode.PercentOutput, 0.0);
}

private boolean isToppedOut(){
    if (getEncoderPosition() >= Positions.Top.getPosition() && getState() != LiftState.GoingDown) {
        setState(LiftState.ToppedOut);
        setPosition(Positions.Top.position);
        return true;
    }
    else
        return false;
    
}

private boolean isZeroedOut() {
    if (getBottomLimit() && getState() != LiftState.GoingUp) {
        setState(LiftState.BottomedOut);
        setPosition(Positions.Intake.position);
        elevatorLeader.setSelectedSensorPosition(0);
        return true;
    }
    else
        return false;
}

public void configPIDF(double kP, double kI, double kD, double kF) {
    elevatorLeader.config_kP(0, kP, 0);
    elevatorLeader.config_kI(0, kI, 0);
    elevatorLeader.config_kD(0, kD, 0);
    elevatorLeader.config_kF(0, kF, 0);
}

/**
 * Set parameters for motion magic control
 *
 * @param cruiseVelocity cruise velocity in sensorUnits per 100ms
 * @param acceleration   cruise acceleration in sensorUnits per 100ms
 */

private int previousCruiseVelocity;
private int previousCruiseAcceleration;

public void configMotionMagic(int cruiseVelocity, int acceleration) {

    //prevents configs from being set the same value multiple times in a loop
    if(previousCruiseVelocity != cruiseVelocity || previousCruiseAcceleration != acceleration)
    {
        elevatorLeader.configMotionCruiseVelocity(cruiseVelocity, 0);
        elevatorLeader.configMotionAcceleration(acceleration, 0);
        previousCruiseVelocity = cruiseVelocity;
        previousCruiseAcceleration = acceleration;
    }
}

public void updatePIDFOnDashboard() {
    SmartDashboard.putNumber("Elevator kP", kP);
    SmartDashboard.putNumber("Elevator kI", kI);
    SmartDashboard.putNumber("Elevator kD", kD);
    SmartDashboard.putNumber("Elevator kF", kF);
}

public void updatePIDFFromDashboard() {
    kP = SmartDashboard.getNumber("Elevator kP", kP);
    kI = SmartDashboard.getNumber("Elevator kI", kI);
    kD = SmartDashboard.getNumber("Elevator kD", kD);
    kF = SmartDashboard.getNumber("Elevator kF", kF);
    configPIDF(kP, kI, kD, kF);
}



  @Override
  public void outputSmartdashboard() 
  {
    //updatePIDFOnDashboard();
    SmartDashboard.putNumber("Desired elevator position", getDesiredPosition());
    SmartDashboard.putNumber("Actual elevator position", getEncoderPosition());
    SmartDashboard.putNumber("Elevator Velocity", getEncoderVeloctiy());
    SmartDashboard.putNumber("Closed loop error", Math.abs(getDesiredPosition() - getEncoderPosition()));
    SmartDashboard.putBoolean("Elevator Bottomed out", getBottomLimit());
  }

  @Override
  public void zeroSensors() 
  {
    this.elevatorLeader.setSelectedSensorPosition(0);
  }

  @Override
  public void resetSubsystem() 
  {
    stopElevator();
  }



  @Override
  public void testSubsystem() {
    boolean sucess = true;

    Timer delay = new Timer();
    System.out.println("///////////////////////////////////////////////////");
    System.out.println("***************Beginning Elevator Test***************");
    Timer.delay(0.2);


    //Test elevator motors
    System.out.println("Testing Elevator Motors and Encoders");
    Timer.delay(0.5);
    TalonChecker checker = new TalonChecker("Elevator Talons", elevatorLeader, false);
    sucess = checker.runTest(2, 100); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Elevator Motors or Encoders***************");
        return;
    }

    if(sucess)
        System.out.println("***************Everything in Elevator is working***************");
    else
        System.out.println("***************Error in Elevator***************");
  }

    
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DirectElevator());
    //setDefaultCommand(new MainElevator());
  }
}


