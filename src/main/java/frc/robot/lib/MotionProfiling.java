/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import jaci.pathfinder.*;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import frc.robot.Constants;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * This class contains the motion profiling controller.  It first loads and initializes trajectories based on waypoints.  If the 
 * trajectory has already been generated before, then it loads a trajectory from a file.  Then, it uses a ramsete follower to 
 * follow the path.
 */

 /*
 notes about this class
 -the trajectory loading from file was not working
 -ramsete is not working and more difficult to work with
 */
public class MotionProfiling 
{
    private double maxVelocity;
    private double absoluteMaxVelocity;
    private double maxAcceleration;
    private double maxJerk;
    private double wheelbase;

    //the loop rate this code is running at.  Robot program runs every 20ms
    private final double dt = 0.05;

    private Trajectory trajectory;

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

     //Should be greater than zero and this increases correction
     private static final double b = 1.5;

     //Should be between zero and one and this increases dampening
     private static final double zeta = 0.7;
 
     //Holds what segment we are on
     private int segmentIndex;
     private Segment current;
 
     //The robot's x and y position and angle
     private Odometry odometry;

     private int leftEncoderStartingPosition;
     private int rightEncoderStartingPosition;
     private double initGyro;
 
 
     //Variable used to calculate linear and angular velocity
     private double lastTheta, nextTheta;
     private double k, thetaError, sinThetaErrorOverThetaError;
     private double desiredAngularVelocity, linearVelocity, angularVelocity;
     private double odometryError;
 
     //Constants
     private static final double EPSILON = 0.00001;
     private static final double TWO_PI = 2 * Math.PI;

    //PID for pathfinder
     private double kD = 0.2;
	 private double kP = 1;
 
     //Variable for holding velocity for robot to drive on
     private double left, right;
     private Output driveOutput;

    public MotionProfiling(double maxVelocity, double maxAcceleration, double maxJerk, double wheelbase)
    {
        this.maxVelocity = maxVelocity;
        this.absoluteMaxVelocity = Constants.maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        this.wheelbase = wheelbase;

        //class used to store the robots x and y position and angle
        odometry = new Odometry();

        driveOutput = new Output();
        
    }


    //Trajectory Methods

    public void loadTrajectory(Waypoint[] path, boolean reverse) {
        Trajectory.Config cfg = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
                dt, maxVelocity, maxAcceleration, maxJerk);

        //Load trajectory from file if it exists, else generate the trajectory and save it 
        String pathHash = String.valueOf(path.hashCode());
        File trajectoryFile = new File("/home/lvuser/paths/" + pathHash + ".csv");
        if (!trajectoryFile.exists()) {
            trajectory = Pathfinder.generate(path, cfg);
            Pathfinder.writeToCSV(trajectoryFile, trajectory);
            System.out.println(pathHash + ".csv not found, wrote to file");
        } else {
            System.out.println(pathHash + ".csv read from file");
            trajectory = Pathfinder.readFromCSV(trajectoryFile);
        }        

        if(reverse)
            trajectory = reversePath(trajectory);

        segmentIndex = 0;
        setInitialOdometry();
    }

    public void loadTrajectoryPathfinder(Waypoint[] path, boolean reverse, int leftEncoderRaw, int rightEncoderRaw, double gyroAngle) {
        Trajectory.Config cfg = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
                dt, maxVelocity, maxAcceleration, maxJerk);

        //Load trajectory from file if it exists, else generate the trajectory and save it 
        String pathHash = String.valueOf(path.hashCode());
        File trajectoryFile = new File("/home/lvuser/paths/" + pathHash + ".csv");
        if (!trajectoryFile.exists()) {
            trajectory = Pathfinder.generate(path, cfg);
            Pathfinder.writeToCSV(trajectoryFile, trajectory);
            System.out.println(pathHash + ".csv not found, wrote to file");
        } else {
            System.out.println(pathHash + ".csv read from file");
            trajectory = Pathfinder.readFromCSV(trajectoryFile);
        }        

        TankModifier modifier = new TankModifier(trajectory);
        
        //this is reversal code from 2018 which is to flip flop the left right trajectories and the encoders always read positive
        if(!reverse) {
            leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
            rightFollower = new EncoderFollower(modifier.getRightTrajectory());
        }
        else 
        {
            leftFollower = new EncoderFollower(modifier.getRightTrajectory());
            rightFollower = new EncoderFollower(modifier.getLeftTrajectory());	
        }
            leftEncoderStartingPosition = leftEncoderRaw;
            rightEncoderStartingPosition = rightEncoderRaw;
            initGyro = gyroAngle;

            leftFollower.configureEncoder(leftEncoderStartingPosition, Constants.ticksPerRotation, Constants.wheelDiameter);
            rightFollower.configureEncoder(rightEncoderStartingPosition, Constants.ticksPerRotation, Constants.wheelDiameter);
            leftFollower.configurePIDVA(kP, 0.0, kD, 1 / absoluteMaxVelocity, 0);
            rightFollower.configurePIDVA(kP, 0.0, kD, 1 / absoluteMaxVelocity, 0);
    }


    //This function is used to invert a trajectory
    private Trajectory reversePath(Trajectory original){
        ArrayList<Segment> segments = new ArrayList<>(Arrays.asList(original.segments));
        Collections.reverse(segments);

        double distance = segments.get(0).position;

        return new Trajectory(segments.stream()
                .map(segment -> new Segment(segment.dt, segment.x, segment.y, distance - segment.position, -segment.velocity, -segment.acceleration, -segment.jerk, segment.heading))
                .toArray(Segment[]::new));
    }





    //Ramsete Methods

     public Output getNextDriveSignal(boolean reverse, double leftEncoderMeters, double rightEncoderMeters, double gyroAngle, boolean inRadians){

        int inverted = 1;

         if(isFinished()){
             driveOutput.updateOutput(0, 0);
 
             return driveOutput;
         }

         if(reverse)
            inverted = -1;
        
 
         left = 0;
         right = 0;

         if(segmentIndex == 0)
         {
             odometry.setInitialRobotState(leftEncoderMeters, rightEncoderMeters, gyroAngle, inRadians);
         }

         odometry.updateRobotOdometry(leftEncoderMeters, rightEncoderMeters, gyroAngle, inRadians);
 
         current = trajectory.get(segmentIndex);
 
         desiredAngularVelocity = calculateDesiredAngular();
 
         linearVelocity = calculateLinearVelocity(current.x, current.y, current.heading, current.velocity, desiredAngularVelocity);
         angularVelocity = calculateAngularVelocity(current.x, current.y, current.heading, current.velocity, desiredAngularVelocity);
 
         left = (-(angularVelocity * wheelbase) + (2 * linearVelocity)) / 2;
         right = ((angularVelocity * wheelbase) + (2 * linearVelocity)) / 2;
 
         driveOutput.updateOutput(left * inverted, right * inverted);
 
         segmentIndex++;
 
         return driveOutput;
     }
 
     private double calculateDesiredAngular(){
         if(segmentIndex < trajectory.length() - 1){
             lastTheta = trajectory.get(segmentIndex).heading;
             nextTheta = trajectory.get(segmentIndex + 1).heading;
             return (nextTheta - lastTheta) / trajectory.get(segmentIndex).dt;
         }else{
             return 0;
         }
     }
 
     private double calculateLinearVelocity(double desiredX, double desiredY, double desiredTheta, double desiredLinearVelocity, double desiredAngularVelocity){
         k = calculateK(desiredLinearVelocity, desiredAngularVelocity);
         thetaError = boundHalfRadians(desiredTheta - odometry.getThetaInRadians());
         odometryError = calculateOdometryError(odometry.getThetaInRadians(), desiredX, odometry.getX(), desiredY, odometry.getY());
         return (desiredLinearVelocity * Math.cos(thetaError)) + (k * odometryError);
     }
 
     private double calculateAngularVelocity(double desiredX, double desiredY, double desiredTheta, double desiredLinearVelocity, double desiredAngularVelocity){
         k = calculateK(desiredLinearVelocity, desiredAngularVelocity);
         thetaError = boundHalfRadians(desiredTheta - odometry.getThetaInRadians());
 
         if(Math.abs(thetaError) < EPSILON){
             //This is for the limit as sin(x)/x approaches zero
             sinThetaErrorOverThetaError = 1;
         }else{
             sinThetaErrorOverThetaError = Math.sin(thetaError)/thetaError;
         }
 
         odometryError = calculateOdometryError(odometry.getThetaInRadians(), desiredX, odometry.getX(), desiredY, odometry.getY());
 
         return desiredAngularVelocity + (b * desiredLinearVelocity * sinThetaErrorOverThetaError * odometryError) + (k * thetaError);
     }
 
     private double calculateOdometryError(double theta, double desiredX, double x, double desiredY, double y){
         return (Math.cos(theta) * (desiredX - x)) + (Math.sin(theta) * (desiredY - y));
     }
 
     private double calculateK(double desiredLinearVelocity, double desiredAngularVelocity){
         //Calculates k from equation 5.12
         return 2 * zeta * Math.sqrt(Math.pow(desiredAngularVelocity, 2) + (b * Math.pow(desiredLinearVelocity, 2)));
     }
 
     private double boundHalfRadians(double radians){
         while (radians >= Math.PI) radians -= TWO_PI;
         while (radians < -Math.PI) radians += TWO_PI;
         return radians;
     }
 
     public void setInitialOdometry(){
         odometry.setInitialRobotOdometry(trajectory.get(0).x, trajectory.get(0).y, trajectory.get(0).heading, true);
     }
 
     public boolean isFinished(){
         return segmentIndex == trajectory.length();
     }

     //a percentage of how much of the path has been followed
     public double percentageDone()
     {
        return (segmentIndex / trajectory.length()) * 100;
     }




     //Pathfinder Methods
    
    public Output followTrajectoryPathfinder(boolean reverse, int leftEncoderRaw, int rightEncoderRaw, double gyroAngle)
    {
        int inverted = 1;

        if(reverse)
            inverted = -1;
        
        //
		int leftEncoderPosition = leftEncoderStartingPosition
				+ Math.abs(leftEncoderStartingPosition - leftEncoderRaw);
		int rightEncoderPosition = rightEncoderStartingPosition
                + Math.abs(rightEncoderStartingPosition - rightEncoderRaw);
                
        gyroAngle = gyroAngle - initGyro;


		double l = leftFollower.calculate(leftEncoderPosition);
		double r = rightFollower.calculate(rightEncoderPosition);

		double gyro_heading = gyroAngle * -1; // counter clockwise rotation should be positive
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

        driveOutput.updateOutput(l * inverted + turn , l * inverted + turn);

		return driveOutput;
    }

    public boolean isPathfinderPathDone()
    {
        return leftFollower.isFinished() && rightFollower.isFinished();
    }
    
 }

