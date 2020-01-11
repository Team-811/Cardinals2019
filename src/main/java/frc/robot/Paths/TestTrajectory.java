/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Paths;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * Add your docs here.
 */
public class TestTrajectory 
{
    public static Waypoint[] testPath = new Waypoint[]{
        new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
        new Waypoint(5, 0.0, Pathfinder.d2r(0.0))
};

}
