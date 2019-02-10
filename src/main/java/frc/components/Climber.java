// JACOB CAZABON <3 SAINTS BOT 2018

package frc.components;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Manages deploying and retracting the climber.
 * 
 * @author Hunter M. 
 */
public class Climber
{
	// Motor speed for climbing
	private final double CLIMB_SPD = 1d;
	
	// Climb motor CAN index (both motors on same controller)
	private final int CLIMBER_CAN = 7;
	
	// Climb motors
	private TalonSRX reacher;
	
	
	/**
	 * Constructor.
	 */
	public Climber()
	{
		// Climb motors (aptly named by Scott)
		this.reacher = new TalonSRX(CLIMBER_CAN);
		
	}
	
	/**
	 * Operates the climber based on button input.
	 * 
	 * @param matt The Matt operating the robot
	 */
	public void control(MattDupuis matt, Lift lift)
	{
		// Climber action
		MattDupuis.Climber climb = matt.getClimber();
		
		
		// If the robot is retracting the climber and is finished deploying:
		if (climb == MattDupuis.Climber.Climb)
		{
			this.reacher.set(ControlMode.PercentOutput, CLIMB_SPD);
		}
		else
		{
			this.reacher.set(ControlMode.PercentOutput, 0d);
		}
	}
}