/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * This is the subsystem interface.  It ensures that the subsystem classes all must contain these methods for safety 
 * and convienance.
 */
public interface ISubsystem 
{
    public abstract void outputSmartdashboard();
    public abstract void zeroSensors();
    public abstract void resetSubsystem();
    public abstract void testSubsystem();
}
