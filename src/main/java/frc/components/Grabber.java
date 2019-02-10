// Brandon Mailloux 2019 - 4688 Saints Bot		#Based on Jacob 2018

package frc.components;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;

/**
 * Manages the Grabber and pnumatic system.
 * 
 * @author Brandon
 */
public class Grabber
{
	// Intake/eject speed factor
	private final double BASE_INTAKE_SPD = -1d;
	
	// Intake motor CAN indices
	private final int GRABBER_VICTOR_VALUE = 7;
	
	// Intake motors
	private VictorSPX intakeWheel;	
	/**
	 * Constructor.
	 */
	public Grabber()
	{
		// Intake motor
		this.intakeWheel = new VictorSPX(GRABBER_VICTOR_VALUE);
	}
	
	/**
	 * Sets the intake motor speeds; -1 means full speed eject and 1 means full
	 * speed intake.
	 * 
	 * @param spd Intake speed, between -1 and 1
	 */
	public void setIntakeSpd(double spd)
	{
		// Clamp speeds to +/-100%
		spd = Math.min(Math.max(spd, -1d), 1d);
		
		// Set speeds
		this.intakeWheel.set(ControlMode.PercentOutput, spd);
	}
	
	/**
	 * Sets drive motor speeds based on driver input.
	 * 
	 * @param matt The Matt that is driving the robot
	 */
	public void control(MattDupuis matt)
	{
		// Intake speeds
		double intake = matt.getIntake() * BASE_INTAKE_SPD;
		this.setIntakeSpd(intake);
	}
}