/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.
 */


public class Elevator extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
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

private volatile LiftState state = LiftState.Stationary;

private TalonSRX elevatorLeader = new TalonSRX(RobotMap.ELEVATOR_MAIN);
private TalonSRX elevatorSlave = new TalonSRX(RobotMap.ELEVATOR_SLAVE);

private double kP = 0.5; // .3
private double kI = 0.0;
private double kD = 4.0; // 4.0
private double kF = 0.1165 * 2; // 0.1165 * 2

//81.2 inches per/s
//Gear is 42:26 geared up
//4096 ticks per rev
//233 ticks per inch
private static final int CRUISE_VELOCITY = 17600; // 1024
private static final int CRUISE_ACCELERATION = 11000; // 1024
private static final int CRUISE_VELOCITY_DOWN = (int) (CRUISE_VELOCITY * 0.7); // 1024
private static final int CRUISE_ACCELERATION_DOWN = (int) (CRUISE_ACCELERATION * 0.6); // 1024

public enum Positions {
    //TODO
    Intake(300),
    Level2Hatch(50000),
    Level3Hatch(22000),
    Level1Cargo(70000),
    Level2Cargo(75001),
    Level3Cargo(0),
    Top(75000);
    private int position;

    Positions(int encPos) {
        this.position = encPos;
    }

    public int getPosition() {
        return this.position;
    }
}

private Positions position = Positions.Intake;

public Positions getPosition() {
    return position;
}

private void setPosition(Positions newPos) {
    this.position = newPos;
}

private final int MOTION_MAGIC_TOLERANCE = 150;
private static final double ELEVATOR_HI_POW = 1.0;
private static final double ELEVATOR_LOW_POW = -ELEVATOR_HI_POW;

public Elevator() {
    //configure motors
    this.elevatorSlave.follow(elevatorLeader);
    this.elevatorLeader.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    this.elevatorLeader.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    this.elevatorLeader.configNominalOutputForward(0.0, 0);
    this.elevatorLeader.configNominalOutputReverse(0.0, 0);
    this.elevatorSlave.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    this.elevatorSlave.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    this.elevatorSlave.configNominalOutputForward(0.0, 0);
    this.elevatorSlave.configNominalOutputReverse(0.0, 0);

    //Encoder
    this.elevatorLeader.setSensorPhase(true);
    this.elevatorLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    zeroSensors();

    //Neutral mode
    this.elevatorLeader.setNeutralMode(NeutralMode.Brake);
    this.elevatorSlave.setNeutralMode(NeutralMode.Brake);

    this.elevatorLeader.setInverted(false);
    this.elevatorSlave.setInverted(false);

    configPIDF(kP, kI, kD, kF);
    configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);

}




public int getEncoderPosition() {
    return elevatorLeader.getSelectedSensorPosition(0);
}

public void startMotionMagic(Positions pos) { 
    //If the desired position is higher than the current, then the elevator must go up
    if (getEncoderPosition() > pos.getPosition()) {
        setState(LiftState.GoingUp);
        configMotionMagic(CRUISE_VELOCITY_DOWN, CRUISE_ACCELERATION_DOWN);
    
    //If the desired position is lower than the current, then the elevator must going down
    } else if (getEncoderPosition() < pos.getPosition()) {          
        setState(LiftState.GoingDown);
        configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);
    }

    elevatorLeader.set(ControlMode.MotionMagic, pos.getPosition());
}

public void checkMotionMagicTermination(Positions pos) {
    if (pos == Positions.Intake) {
        if (getEncoderPosition() <= (MOTION_MAGIC_TOLERANCE * 2)) {
            state = LiftState.Stationary;
            stopElevator();
            position = pos;
        }
    } else if (Math.abs(pos.getPosition() - getEncoderPosition()) <= MOTION_MAGIC_TOLERANCE) {
        state = LiftState.Stationary;
        stopElevator();
        position = pos;
    }
    SmartDashboard.putString("Desired elevator position enum", pos.toString());
    SmartDashboard.putNumber("Motion Magic set position", elevatorLeader.getClosedLoopTarget(0));
    SmartDashboard.putNumber("CTRError", elevatorLeader.getClosedLoopError(0));
    SmartDashboard.putNumber("Desired elevator position", pos.getPosition());
    SmartDashboard.putNumber("Closed loop error", Math.abs(pos.getPosition() - getEncoderPosition()));
}

public void stopElevator() {
    elevatorLeader.set(ControlMode.PercentOutput, 0.0);
}

public void directElevate(double pow) {
    if (getState() == LiftState.BottomedOut && pow < 0.0) {
        return;
    }
    if (getState() == LiftState.ToppedOut && pow > 0.0) {
        return;
    }
    if (pow > 0.0) {
        setState(LiftState.GoingUp);
    }
    if (pow < 0.0) {
        setState(LiftState.GoingDown);
    }
    if (pow == 0.0) {
        setState(LiftState.Stationary);
    }
    elevatorLeader.set(ControlMode.PercentOutput, pow);
}

private void checkIfToppedOut() {
    if (getEncoderPosition() >= Positions.Top.getPosition() && getState() != LiftState.GoingDown) {
        setState(LiftState.ToppedOut);
        setPosition(Positions.Top);
        stopElevator();
    }
}

private void checkIfZeroedOut() {
    if (getEncoderPosition() <= Positions.Intake.getPosition() && getState() != LiftState.GoingUp) {
        setState(LiftState.BottomedOut);
        setPosition(Positions.Intake);
        stopElevator();
    }
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


public void configMotionMagic(int cruiseVelocity, int acceleration) {
    elevatorLeader.configMotionCruiseVelocity(cruiseVelocity, 0);
    elevatorLeader.configMotionAcceleration(acceleration, 0);
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
    updatePIDFOnDashboard();
  }

  @Override
  public void zeroSensors() 
  {
    this.elevatorLeader.setSelectedSensorPosition(0, 0, 0);
  }

  @Override
  public void resetSubsystem() 
  {
    elevatorLeader.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void checkSubsystem()
  {
    
  }

  @Override
  public void testSubsystem() {
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}


