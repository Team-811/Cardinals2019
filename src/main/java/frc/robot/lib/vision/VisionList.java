/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.vision;

import java.util.ArrayList;

/**
 * Contains a list of vision targets and a timestamp of when the targets were
 * recieved
 */
public class VisionList 
{
    private ArrayList<VisionTarget> targets;
    private double timestamp;

    //Constructors
    public VisionList()
    {
        targets = new ArrayList<>();
        this.timestamp = 0;
    }

    public VisionList(double timestamp)
    {
        targets = new ArrayList<>();
        this.timestamp = timestamp;
    }

    //Methods

    public ArrayList<VisionTarget> getTargets()
    {
        return targets;
    }

    public double getTimestamp()
    {
        return timestamp;
    }

    public void setTimestamp(double timestamp)
    {
        this.timestamp = timestamp;
    }

    public void clearList()
    {
        targets.clear();
    }

    public void addTarget(VisionTarget target)
    {
        targets.add(target);
    }




    


}
