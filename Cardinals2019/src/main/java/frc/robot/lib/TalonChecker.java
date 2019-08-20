/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class is used to test whether talons are working or not.  There are two modes, a non encoder and encoder mode.  The non encoder looks at just the current 
 * and the encoder version looks at current, RPM, and sensor phase.  It will first check if the sensor is even present before actually moving the motor.  Written by team 195
 */
public class TalonChecker 
{
    
    private static final String TEST_NOT_RUN_STR = "Test not run yet!";

	private double mTestDurationSeconds = 1.5;
	private double mMotorSpeed = 0.75;
	private TalonSRX testingSpeedController;
    private String motorName;
	private double motorRPM;
	private double motorCurrent;
	private boolean inverted = false;
	private boolean sensorInPhase = false;
    private boolean testCompleted = false;
    private UnitConverter converter = new UnitConverter();

	public TalonChecker(String motorName, TalonSRX testingSpeedController, boolean inverted) {
		this.testingSpeedController = testingSpeedController;
        this.motorName = motorName;
        this.inverted = inverted;
	}

	//A test for a motor without an encoder
	public boolean runTest(double currentThreshold) {
		// testingSpeedController.configForwardSoftLimitEnable(false, 20);
		// testingSpeedController.configReverseSoftLimitEnable(false, 20);
        if(testingSpeedController.getFirmwareVersion() == 0)
        {
            System.out.println(motorName + "is not present!!!!!");
            return false;
        }
        else
        {
            //Setup
            double motorPositionPreTest = 0;
            double motorPositionPostTest = 0;
		    mMotorSpeed = Math.abs(mMotorSpeed);
		    testingSpeedController.set(ControlMode.PercentOutput, inverted ? -mMotorSpeed : mMotorSpeed);
            Timer.delay(mTestDurationSeconds/2.0);
        
            //Mid Spin
            motorCurrent = testingSpeedController.getOutputCurrent();
            Timer.delay(mTestDurationSeconds/2.0);
        
            //End
			testingSpeedController.set(ControlMode.PercentOutput, 0);
			
            testCompleted = true;
        
            //Analysis
            if(isCurrentUnderThreshold(currentThreshold))
                return false;
            else
                return true;
            
		}

	}

	//A test for a motor with an encoder
	public boolean runTest(double currentThreshold, double RPMThreshold) {
		// testingSpeedController.configForwardSoftLimitEnable(false, 20);
		// testingSpeedController.configReverseSoftLimitEnable(false, 20);
        if(testingSpeedController.getFirmwareVersion() == 0)
        {
            System.out.println(motorName + "is not present!!!!!");
            return false;
        }
        else
        {
            //Setup
            double motorPositionPreTest = 0;
            double motorPositionPostTest = 0;
		    motorPositionPreTest = getPosition();
		    mMotorSpeed = Math.abs(mMotorSpeed);
		    testingSpeedController.set(ControlMode.PercentOutput, inverted ? -mMotorSpeed : mMotorSpeed);
            Timer.delay(mTestDurationSeconds/2.0);
        
            //Mid Spin
            motorCurrent = testingSpeedController.getOutputCurrent();
		    motorRPM = getRPM();
            motorPositionPostTest = getPosition();
            Timer.delay(mTestDurationSeconds/2.0);
        
            //End
		    testingSpeedController.set(ControlMode.PercentOutput, 0);
            if (inverted & (motorPositionPostTest < motorPositionPreTest))
			    sensorInPhase =  true;
		    else if (!inverted & (motorPositionPostTest > motorPositionPreTest))
			    sensorInPhase = true;
		    else
				sensorInPhase = false;
				
            testCompleted = true;
        
            //Analysis
            
            if(isCurrentUnderThreshold(currentThreshold) || isRPMUnderThreshold(RPMThreshold))
                return false;
            else
				return true;
				
        }
            
	}

	public boolean isSensorInPhase() {
		if (testCompleted)
			return sensorInPhase;
		else {
			System.out.println("Encoder out of phase");
			return false;
		}
	}

	private double getPosition() {
		return converter.ticksToRotations(testingSpeedController.getSelectedSensorPosition(0), 1024);
	}

	private double getRPM() {
		return converter.talonUnitsToRPM(testingSpeedController.getSelectedSensorVelocity(0), 1024);
	}

	public void setZero() {
		testingSpeedController.set(ControlMode.PercentOutput, 0);
	}

	public String getMotorName() {
		return motorName;
	}

	public double getMotorCurrent() {
		if (testCompleted)
			return motorCurrent;
		else {
			System.out.println(motorName + " has not completed test yet");
			return 0;
		}
	}

	public double getMotorRPM() {
		if (testCompleted)
			return motorRPM;
		else {
			System.out.println(motorName + " has not completed test yet");
			return 0;
		}
	}

	public boolean isCurrentUnderThreshold(double threshold) {
        if(motorCurrent < threshold || !testCompleted)
        {
            System.out.println(motorName + " has low current!!!!!");
            return true;
        }
        else 
            return false;
	}

	public boolean isRPMUnderThreshold(double threshold) {
        if (motorRPM < threshold || !testCompleted)
        {
            System.out.println(motorName + " has low RPM!!!!!");
            return true;
        }
		else {
			return false;
		}
	}

}
