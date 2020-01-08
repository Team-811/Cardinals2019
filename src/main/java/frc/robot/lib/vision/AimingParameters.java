/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.vision;

import frc.robot.lib.geometry.Rotation2d;

/**
 * This gives information about the location of a vision target based on the distance from the robot to the goal and the angle from the robot to the goal
 */
public class AimingParameters {
    double range;
    double last_seen_timestamp;
    double stability;
    Rotation2d robot_to_goal;

    public AimingParameters(double range, Rotation2d robot_to_goal, double last_seen_timestamp,
            double stability) {
        this.range = range;
        this.robot_to_goal = robot_to_goal;
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getRobotToGoal() {
        return robot_to_goal;
    }

    public double getX()
    {
        return range * Math.cos(robot_to_goal.getRadians());
    }

    public double getY()
    {
        return range * Math.sin(robot_to_goal.getRadians());
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }
}
