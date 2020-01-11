/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

/**
 * This class is used to convert common units in FRC.
 */
public class UnitConverter 
{
    public static double metersPerSecondToFeetPerSecond(double metersPerSecond)
    {
        return metersPerSecond * 3.281;
    }

    public static double feetPerSecondToMetersPerSecond(double feetPerSecond)
    {
        return feetPerSecond / 3.281;
    }



    //A talon unit is 1 tick per 100ms

    public static double talonUnitsToMetersPerSecond(double talonUnits, double wheelDiameterInMeters, double EncoderTicksPerRotation)
    {
        return talonUnits * 10 / EncoderTicksPerRotation * (Math.PI * wheelDiameterInMeters);
    }

    public static double metersPerSecondToTalonUnits(double metersPerSecond, double wheelDiameterInMeters, double EncoderTicksPerRotation)
    {
        return metersPerSecond / 10 * EncoderTicksPerRotation / (Math.PI * wheelDiameterInMeters);
    }

    public static double talonUnitsToRotationsPerSecond(double talonUnits, double EncoderTicksPerRotation)
    {
        return talonUnits * 10 / EncoderTicksPerRotation;
    }

    public static double rotationsPerSecondToTalonUnits(double rotations, double EncoderTicksPerRotation)
    {
        return rotations / 10 * EncoderTicksPerRotation;
    }

    public static double talonUnitsToRPM(double talonUnits, double EncoderTicksPerRotation)
    {
        return talonUnitsToRotationsPerSecond(talonUnits, EncoderTicksPerRotation) / 60;
    }   

    public static double RPMToTalonUnits(double talonUnits, double EncoderTicksPerRotation)
    {
        return rotationsPerSecondToTalonUnits(talonUnits, EncoderTicksPerRotation) * 60;
    }

    public static double ticksToMeters(double ticks, double EncoderTicksPerRotation, double wheelDiameterInMeters)
    {
        return ticks / EncoderTicksPerRotation * (Math.PI * wheelDiameterInMeters);
    }

    public static double metersToTicks(double meters, double EncoderTicksPerRotation, double wheelDiameterInMeters)
    {
        return meters * EncoderTicksPerRotation / (Math.PI * wheelDiameterInMeters);
    }

    public static double ticksToRotations(double ticks, double EncoderTicksPerRotation)
    {
        return ticks / EncoderTicksPerRotation;
    }

    public static double rotationsToTicks(double meters, double EncoderTicksPerRotation)
    {
        return meters * EncoderTicksPerRotation;
    }

}
