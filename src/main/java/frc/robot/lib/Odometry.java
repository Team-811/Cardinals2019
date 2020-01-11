/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

/**
 * This class is used to store a X coordinate, a Y coordinate, and an angle of something.  Also, it is used to store the robot's
 * position on the field based on its starting position, encoder values, and gyro values
 */

public class Odometry 
{
    private double xCoordinate;
    private double yCoordinate;
    private double theta; //in radians

    private double prevEncoderLeft;
    private double prevEncoderRight;
    private double prevGyroAngle;

    //Default Constructor
    public Odometry()
    {
        xCoordinate = 0;
        yCoordinate = 0;
        theta = 0;


    }

    //Parameterized constructors
    public Odometry(double xCoordinate, double yCoordinate, double theta, boolean inRadians)
    {
        if(!inRadians)
        {
            theta = Math.toRadians(theta);
        }

        this.xCoordinate = xCoordinate;
        this.yCoordinate = yCoordinate;
        this.theta = theta;
    }

    
    //The following deal with basic coordinate storage 


    //Getter methods

    public double getX ()
    {
        return xCoordinate;
    }

    public double getY()
    {
        return yCoordinate;
    }

    public double getThetaInRadians()
    {
        return theta;
    }

    public double getThetaInDegrees()
    {
        return Math.toDegrees(theta);
    }



    //Setter Methods

    public void setX(double xCoordinate)
    {
        this.xCoordinate = xCoordinate;
    }

    public void setY(double yCoordinate)
    {
        this.yCoordinate = yCoordinate;
    }

    public void setThetaRadians(double thetaRadians)
    {
        this.theta = thetaRadians;
    }

    public void setThetaDegrees(double thetaDegrees)
    {
        this.theta = Math.toRadians(thetaDegrees);
    }







    //The following deal with the position of the robot


    //Setting Odometries based on robot sensors

    public void updateRobotOdometry(double encoderLeft, double encoderRight, double gyroAngle, boolean inRadians)
    {
        if(!inRadians)
        {
            gyroAngle = Math.toRadians(gyroAngle);
        }

        //summation of the average change in the encoders and broken into components for the directions
        this.xCoordinate += (Math.cos(gyroAngle) * ((getDeltaEncoderLeft(encoderLeft) + getDeltaEncoderRight(encoderRight))/2));
        this.yCoordinate += (Math.sin(gyroAngle) * ((getDeltaEncoderLeft(encoderLeft) + getDeltaEncoderRight(encoderRight))/2));

        this.theta = getDeltaGyro(gyroAngle); 

        this.prevEncoderLeft = encoderLeft;
        this.prevEncoderRight = encoderRight;

    }

    public void setInitialRobotOdometry(double x, double y, double theta, boolean inRadians)
    {
        if(!inRadians)
        {
            theta = Math.toRadians(theta);
        }

        setX(x);
        setY(y);
        setThetaRadians(theta);
    }

    public void setInitialRobotState(double initialEncoderLeft, double initialEncoderRight, double initialGyroAngle, boolean inRadians)
    {
        if(!inRadians)
        {
            initialGyroAngle = Math.toRadians(theta);
        }

        this.prevEncoderLeft = initialEncoderLeft;
        this.prevEncoderRight = initialEncoderRight;
        this.prevGyroAngle = Math.toRadians(initialGyroAngle);

    }

    private double getDeltaEncoderLeft(double encoderLeft)
    {
        return encoderLeft - prevEncoderLeft;
    }

    private double getDeltaEncoderRight(double encoderRight)
    {
        return encoderRight - prevEncoderRight;
    }

    private double getDeltaGyro(double gyroAngle)
    {
        return gyroAngle - prevGyroAngle;
    }


}
