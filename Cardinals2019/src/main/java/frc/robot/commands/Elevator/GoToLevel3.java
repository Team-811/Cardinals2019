/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Positions;

/**
 * This is a command.  A command is used to make actual actions happen on the robot.  It can be a single action or a sequence of actions.
 */
public class GoToLevel3 extends InstantCommand {
  /**
   * Add your docs here.
   */
  public GoToLevel3() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
    requires(Robot.intakes);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.intakes.isCargoIntakeDown())
      Robot.elevator.setPosition(Positions.Level3Cargo.getPosition());
    else
      Robot.elevator.setPosition(Positions.Level3Hatch.getPosition());
  }

}
