/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intakes.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Intakes.InstantCommands.*;
import frc.robot.commands.Utility.*;
import frc.robot.commands.LED.*;

public class OverTheTopOuttakeComp extends CommandGroup {
  /**
 * This is a command.  A command is used to make actual actions happen on the robot.  It can be a single action or a sequence of actions.  This one is a combination of commands
 * that allows the robot to release a ball over the top
 */
  public OverTheTopOuttakeComp() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    //Play outtake LED pattern
    addSequential(new Rainbow());
    //Just to ensure that the intake is down even though it should be down anyway
    addSequential(new DropCargoIntake());
    //Pulse the motors a few times and bring the ball over the top
    addSequential(new Unjam());
    addSequential(new TimerCommand(1.2));
    //Punch the ball
    addSequential(new ExtendHatchIntake());
    addSequential(new TimerCommand(0.15));
    addSequential(new RetractHatchIntake());
    addSequential(new TimerCommand(0.6));
    //Return to default state
    addSequential(new StopCargo());
    addSequential(new ReleaseHatch());
    addSequential(new BringUpCargoIntake());
    addSequential(new RedFlame());
  }
}
