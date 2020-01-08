/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SerialPort;

/**
 *This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  This subsystem controls the LEDS.  All it does is take in a number for each 
 *pattern and it will spit that number onto the usb serial port for the arduino to handle
 */
public class LED extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LED instance = new LED();

  public static LED getInstance() {
    return instance;
  }

  private SerialPort LEDPort;


  public LED()
  {
    try{
    LEDPort = new SerialPort(9600, SerialPort.Port.kUSB);
    }
    catch(Exception e)
    {
      //ignore
    }

  }

  public void setLEDs(int preset)
  {
      try{
        switch(preset)
        {
          //RainbowDotTrail
          case 1:
            LEDPort.writeString("1");
            break;
          //Rainbow
          case 2:
            LEDPort.writeString("2");
            break;
          //RainbowGradient
          case 3:
            LEDPort.writeString("3");
            break;
          //RainbowSnake
          case 4:
            LEDPort.writeString("4");
            break;
          //RedYellowSnake
          case 5:
            LEDPort.writeString("5");
            break;
          //BlueYellowSnake
          case 6:
            LEDPort.writeString("6");
            break;
          //Confetti
          case 7:
            LEDPort.writeString("7");
            break;
          //America
          case 8:
            LEDPort.writeString("8");
            break;
          //RedFlame
          case 9:
            LEDPort.writeString("9");
            break;
          //BlueFlame
          case 10:
            LEDPort.writeString("10");
            break;
          //BGFlame
          case 11:
            LEDPort.writeString("11");
            break;
          //RainbowFlame
          case 12:
            LEDPort.writeString("12");
            break;
          //ChaseRainbow
          case 13:
            LEDPort.writeString("13");
            break;
          //Red
          case 14:
            LEDPort.writeString("14");
            break;
          //Green
          case 15:
            LEDPort.writeString("15");
            break;
        }
      }
      catch(Exception e) 
      {
        //ignore
      }
    }
  


  @Override
  public void outputSmartdashboard() 
  {
    
  }

  @Override
  public void zeroSensors() 
  {
    
  }

  @Override
  public void resetSubsystem() 
  {
    LEDPort.reset();
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
