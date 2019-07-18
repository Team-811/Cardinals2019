/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.OI;
import frc.robot.subsystems.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static Drivetrain drivetrain;
  public static Intakes intakes;
  public static Climber climber;
  public static Elevator elevator;
  public static LED led;
  public static Vision vision;
  public static RobotStateEstimator stateEstimator;
  public static OI controllers; 

  //Command m_autonomousCommand;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //initialize all the subsystems
    //Retrieves the instance from the class to make sure there arent 2 drivetrains etc
    drivetrain = Drivetrain.getInstance();
    intakes = Intakes.getInstance();
    climber = Climber.getInstance();
    elevator = Elevator.getInstance();
    led = LED.getInstance();
    vision = Vision.getInstance();
    stateEstimator = RobotStateEstimator.getInstance();
    controllers = OI.getInstance();

    updateSmartdashboard();
    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);

    //CameraServer for SmartDashboard
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);

    //Start threads
    //Threads are used for leds, vision, and position tracker so they dont overload the main robot program
    Thread updateThread = new Thread(new RunUpdater());
    updateThread.start();


    //Presets the cargo intake to the up position
    Robot.intakes.bringUpCargoIntake();

    //Presets the LEDs to play the flame patten
    led.setLEDs(9); //Red Flame

    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
      updateSmartdashboard();
      
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
    //m_autonomousCommand.start();
    //}

    updateSmartdashboard();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    updateSmartdashboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
   // if (m_autonomousCommand != null) {
   //   m_autonomousCommand.cancel();
    //}

      updateSmartdashboard();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    updateSmartdashboard();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is enabled in test mode
   */
  @Override
  public void testInit() {
    //Initial Print
    System.out.println("/////////////////////////////////////////////////////");
    System.out.println("***********Robot has now entered test mode***********");
    System.out.println("*****Warning: Robot will move during this period*****");
    System.out.println("*******Please move out of the way of the Robot*******");
    System.out.println("/////////////////////////////////////////////////////");
    Timer.delay(5);

    //Run tests
    drivetrain.testSubsystem();
    elevator.testSubsystem();
    intakes.testSubsystem();


    
  }


  private void updateSmartdashboard()
  {
      drivetrain.outputSmartdashboard();
      elevator.outputSmartdashboard();
      intakes.outputSmartdashboard();
      //led.outputSmartdashboard();
      vision.outputSmartdashboard();
  }





  public class RunUpdater implements Runnable
  {
    public void run()
    {
      while (true) {
        try {
          Thread.sleep(20L);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }

        ledOnLoop();
        //visionOnLoop();

      }
    }
  }


  public void ledOnLoop()
  {
      led = LED.getInstance();
  }

  public void visionOnLoop()
  {
      vision = vision.getInstance();
      if(vision != null)
        vision.onLoop();
    
  }

  public void updatePositionOnLoop()
  {
      stateEstimator.runRobotStateEstimator(Timer.getFPGATimestamp());
  }

}
