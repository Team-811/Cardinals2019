/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  
  //Drivetrain

  public static final int DRIVE_TOP_LEFT_MOTOR     = 0;
  public static final int DRIVE_TOP_RIGHT_MOTOR    = 1;
  public static final int DRIVE_BOTTOM_LEFT_MOTOR  = 2;
  public static final int DRIVE_BOTTOM_RIGHT_MOTOR = 3;
  public static final SerialPort.Port GYRO_PORT = SerialPort.Port.kMXP;


  //Intake
  
  //Hatch
  public static final int INTAKE_RETRACT_PISTON = 1;
  public static final int INTAKE_HATCH_PISTON_FORWARD = 4;
  public static final int INTAKE_HATCH_PISTON_REVERSE = 5;
  public static final int INTAKE_LIMIT_SWITCH = 3;

    //Cargo
  public static final int INTAKE_CARGO_PISTON_FORWARD = 2;
  public static final int INTAKE_CARGO_PISTON_REVERSE = 3;
  public static final int INTAKE_CARGO_MOTOR = 4;
  public static final int INTAKE_DISTANCE_SENSOR = 0;


  //Elevator

  public static final int ELEVATOR_MAIN     = 5;
  public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH = 0;


  //Climber
  public static final int CLIMBER_ARM_MOTOR = 6;
  public static final int CLIMBER_STILT_MOTOR = 7;
  public static final int CLIMBER_WHEEL_MOTOR = 8;
  public static final int CLIMBER_ARM_LIMIT_SWITCH = 4;
  public static final int CLIMBER_STILT_LIMIT_SWITCH = 1;
  public static final int CLIMBER_HAB_LIMIT_SWITCH = 3;


  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
