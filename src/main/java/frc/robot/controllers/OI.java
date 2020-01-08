/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import frc.robot.controllers.Utility.*;
import frc.robot.commands.Climber.single.MoveRobot;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.GoToBottom;
import frc.robot.commands.Elevator.GoToLevel1;
import frc.robot.commands.Elevator.GoToLevel2;
import frc.robot.commands.Elevator.GoToLevel3;
import frc.robot.commands.Intakes.CommandGroups.*;
import frc.robot.commands.Intakes.InstantCommands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

    private static OI instance = new OI();

    public static OI getInstance() {
      return instance;
    }

    public BobXboxController driveController;
    public BobXboxController operatorController;

    public OI()
    {
      driveController = new BobXboxController(0, 0.2, 0.2);
      operatorController = new BobXboxController(1, 0.2, 0.2);

      //Driver

      driveController.rightBumper.whenPressed(new ToggleDriveMode());
      driveController.leftBumper.whileHeld(new SlowMode());
      driveController.startButton.whileHeld(new MoveRobot());
      driveController.leftTriggerButton.whenPressed(new AquireHatchComp());
      driveController.rightTriggerButton.whenPressed(new PlaceHatchComp());
      //Drive left joy y axis is defined in the DriveWithJoy command for forward motion of drivetrain
      //Drive left joy x axis is defined in the DriveWithJoy command for strafing motion of drivetrain
      //Drive right joy x axis is defined in the DriveWithJoy command for rotation of drivetrain



      //Operator

      operatorController.aButton.whenPressed(new GoToBottom());
      operatorController.bButton.whenPressed(new GoToLevel1());
      operatorController.xButton.whenPressed(new GoToLevel2());
      operatorController.yButton.whenPressed(new GoToLevel3());  
      operatorController.rightBumper.whenPressed(new OverTheTopOuttakeComp());
      operatorController.leftBumper.whenPressed(new OuttakeBallComp());
      operatorController.startButton.whenPressed(new IntakeCargoComp());
      operatorController.selectButton.whenPressed(new ResetIntake());
      //Operator trigger twist is defined in default elevator command for manual elevator position
      //Operator left joy y axis is defined in the ClimbWithJoy command for arm motor
      //Operator right joy y axis is defined in the ClimbWithJoy command for stilt motor

    }

  

}
