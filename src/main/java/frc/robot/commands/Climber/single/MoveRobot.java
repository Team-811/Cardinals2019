/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber.single;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * This is a command.  A command is used to make actual actions happen on the robot.  It can be a single action or a sequence of actions.
 */

 
public class MoveRobot extends Command {
  public MoveRobot() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climber.setWheelMotor(0.6);
    Robot.drivetrain.DriveWithJoy(-0.5, 0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;//robot.climber.getHabSwitch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.setWheelMotor(0);
    Robot.drivetrain.DriveWithJoy(0, 0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.climber.setWheelMotor(0);
    Robot.drivetrain.DriveWithJoy(0, 0, 0);
  }
}
