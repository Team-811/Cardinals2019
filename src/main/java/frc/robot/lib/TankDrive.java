/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

/**
 * This class contains different contol modes for a tank drivetrain (a set of wheels or tank treads that are chained together
 * to control the left and right side of the drivetrain only).  Each method takes in parameters like forward movement, rotation, 
 * etc and then calculates what value each motor in the drivetrain is set to.  The control modes that a tank drivetrain has 
 * are tank drive, arcade drive, and curvature drive.
 */

public class TankDrive 
{
    private Output driveOutput;

    //used to invert all drive modes (contains either 1 or -1) 
    private int forwardBackwardInvert;
    private int rotationInvert;

    public TankDrive()
    {
        driveOutput = new Output();

        forwardBackwardInvert = 1;
        rotationInvert = 1;
    }
    
    //This drive mode uses one joystick for each side of wheels (to go straight, you need to push both joysticks forward)
    public Output tankDrive(double leftValue, double rightValue)
    {
        //First invert all the values if needed before doing the motor output calculations
        leftValue = leftValue * forwardBackwardInvert;
        rightValue = rightValue * rotationInvert;

        driveOutput.updateOutput(leftValue, rightValue);

        return driveOutput;
    }

    //This drive mode uses one joystick for forward and backward travel and the other for turning (to go straight, you need to push one joystick forward)
    public Output arcadeDrive(double forwardValue, double rotationValue)
    {
        //First invert all the values if needed before doing the motor output calculations
        forwardValue = forwardValue * forwardBackwardInvert;
        rotationValue = rotationValue * rotationInvert;


        double leftValue = forwardValue + rotationValue;
        double rightValue = forwardValue - rotationValue;

        driveOutput.updateOutput(leftValue, rightValue);

        return driveOutput;
    }

    //TODO
    //This drive mode is like arcade but the turn joystick controls the curvature of the robot and not the turn rate (is not working)
    public Output curvatureMecanumDrive(double forwardValue, double rotationValue, boolean isQuickTurn, boolean isHighGear, double strafeValue, double deadbandValue)
    {
        double m_quickStopThreshold = 0.2;
        double m_quickStopAlpha = 0.1;
        double m_quickStopAccumulator = 0;

        forwardValue = limit(forwardValue, 1);
        forwardValue = handleDeadband(forwardValue, deadbandValue);

        rotationValue = limit(rotationValue, 1);
        rotationValue = handleDeadband(rotationValue, deadbandValue);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(forwardValue) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * limit(rotationValue, 1) * 2;
      }
      overPower = true;
      angularPower = rotationValue;
    } else {
      overPower = false;
      angularPower = Math.abs(forwardValue) * rotationValue - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = forwardValue + angularPower;
    double rightMotorOutput = forwardValue - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }


        driveOutput.updateOutput(leftMotorOutput, rightMotorOutput);

        return driveOutput;
    }

    //Inversion methods
    public void invertForwardBackward(boolean inverted)
    {
        if(inverted)
        {
            forwardBackwardInvert = -1;
        }
        else
        {
            forwardBackwardInvert = 1;
        }
    }

    public void invertRotation(boolean inverted)
    {
        if(inverted)
        {
            rotationInvert = -1;
        }
        else
        {
            rotationInvert = 1;
        }
    }


    //this function will make a number zero if it does not pass a certain threshold, used to prevent small outputs getting through and damaging motors
    private double handleDeadband(double val, double deadband) {
      return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    //this function limits an input between a certain range, used to keeep motor outputs between 1 and -1
    private double limit(double input, double limit)
    {
      if(input > limit )
      {
          return limit;
      }
      else if(input < -limit)
      {
          return -limit;
      }
      else
      {
          return input;
      }
    }
}
