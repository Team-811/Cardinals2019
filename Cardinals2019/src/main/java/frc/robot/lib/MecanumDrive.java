/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

/**
 * This class contains different contol modes for a mecanum drivetrain(4 mecanum wheels that are independently controled).
 * Each method takes in parameters like forward movement, strafe movement, rotation, etc. and then calculates what value 
 * each motor in the drivetrain is set to.  The control modes that a mecanum drivetrain has are tank drive, arcade drive,
 * curvature drive, and field oriented drive.
 */

public class MecanumDrive 
{
    private Output driveOutput;

    private int forwardBackwardInvert;
    private int rotationInvert;
    private int strafingInvert;

    public MecanumDrive()
    {
        driveOutput = new Output();

        forwardBackwardInvert = 1;
        rotationInvert = 1;
        strafingInvert = 1;
    }

    //This drive mode uses one joystick for each side of wheels (to go straight, you need to push both joysticks forward)
    public Output tankMecanumDrive(double leftValue, double rightValue, double strafeValue)
    {
        //First invert all the values if needed before doing the motor output calculations
        leftValue = leftValue * forwardBackwardInvert;
        rightValue = leftValue * rotationInvert;
        strafeValue = strafeValue * strafingInvert;

        double topLeftValue = handleLimits(leftValue + strafeValue);
        double bottomLeftValue = handleLimits(leftValue - strafeValue);
        double topRightValue = handleLimits(rightValue - strafeValue);
        double bottomRightValue = handleLimits(rightValue + strafeValue); 

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }

    //This drive mode uses one joystick for forward and backward travel and the other for turning (to go straight, you need to push one joystick forward)
    public Output arcadeMecanumDrive(double forwardValue, double rotationValue, double strafeValue)
    {
        //First invert all the values if needed before doing the motor output calculations
        forwardValue = forwardValue * forwardBackwardInvert;
        strafeValue = strafeValue * strafingInvert;
        rotationValue = rotationValue * rotationInvert;


        double topLeftValue = handleLimits((forwardValue + rotationValue) + strafeValue);
        double bottomLeftValue = handleLimits((forwardValue + rotationValue) - strafeValue);
        double topRightValue = handleLimits((forwardValue - rotationValue) - strafeValue);
        double bottomRightValue = handleLimits((forwardValue - rotationValue) + strafeValue);

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }

    //TODO
    //This drive mode is like arcade but the turn joystick controls the curvature of the robot and not the turn rate (is not working)
    public Output curvatureMecanumDrive(double forwardValue, double rotationValue, boolean isQuickTurn, boolean isHighGear, double strafeValue, double deadbandValue)
    {
        double m_quickStopThreshold = 0.2;
        double m_quickStopAlpha = 0.1;
        double m_quickStopAccumulator = 0;

        forwardValue = handleLimits(forwardValue);
        forwardValue = handleDeadband(forwardValue, deadbandValue);

        rotationValue = handleLimits(rotationValue);
        rotationValue = handleDeadband(rotationValue, deadbandValue);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(forwardValue) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * handleLimits(rotationValue) * 2;
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

        double topLeftValue = handleLimits(leftMotorOutput + strafeValue);
        double bottomLeftValue = handleLimits(leftMotorOutput - strafeValue);
        double topRightValue = handleLimits(rightMotorOutput - strafeValue);
        double bottomRightValue = handleLimits(rightMotorOutput + strafeValue); 

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }


    //This drive mode is like arcade, but the forward backward travel is with respect to the field and not the robot
    public Output fieldOrientedDrive(double forwardValue, double rotationValue, double strafeValue, double gyroAngle)
    {
        //First invert all the values if needed before doing the motor output calculations
        forwardValue = forwardValue * forwardBackwardInvert;
        strafeValue = strafeValue * strafingInvert;
        rotationValue = rotationValue * rotationInvert;

        //Calculate the correct forward and strafe values based on the robot's heading
        double y = Math.sin(Math.toRadians(gyroAngle));
        double x= Math.cos(Math.toRadians(gyroAngle));
        double forward= forwardValue*x + strafeValue*y;
        double strafe= -forwardValue*y + strafeValue*x;

        //Calculates the output of each motor 
        double topLeftValue = handleLimits((forward + rotationValue) + strafe);
        double bottomLeftValue = handleLimits((forward + rotationValue) - strafe);
        double topRightValue = handleLimits((forward - rotationValue) - strafe);
        double bottomRightValue = handleLimits((forward - rotationValue) + strafe);

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

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

    public void invertStrafing(boolean inverted)
    {
        if(inverted)
        {
            strafingInvert = -1;
        }
        else
        {
            strafingInvert = 1;
        }
    }

    //this function will make a number zero if it does not pass a certain threshold, used to prevent small outputs getting through and damaging motors
    private double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    //this function limits an input between a certain range, used to keeep motor outputs between 1 and -1
    private double handleLimits(double input)
    {
        if(input > 1 )
        {
            return 1;
        }
        else if(input < -1)
        {
            return -1;
        }
        else
        {
            return input;
        }
    }
        
}


    

