/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This class contains constants about the robot like physical dimensions
 */
public class Constants 
{
    //TODO
    public static final double wheelbase = 0;
    public static final double wheelDiameter = 0.1;
    public static final double smallWheelDiameter = 0.1;
    public static final double maxVelocity = 1;
    public static final double maxAcceleration = 1;
    public static final double maxJerk = 50;
    public static final double trackScrubFactor = 1;
    public static final double strafeTrackScrubFactor = 0.8;
    public static final int ticksPerRotation = 1024;

    public static final double robotLength = 0.8763; //With bumpers
    public static final double robotWidth = 0.8636; //With bumpers


    //camera location
    public static final double X_DIR_OFFSET_IN_METERS = .18;
    public static final double Y_DIR_OFFSET_IN_METERS = 0;
    public static final double INITIAL_Z_AXIS_ROTATION = 0;

    //vision stuff
    public static double kMaxGoalTrackAge = 1.0;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;

    public static final double cameraLatency = 0.2; //TODO

}
