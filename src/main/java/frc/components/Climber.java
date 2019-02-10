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
	private final int FL_CLIMB_PORT = 8;
	private final int FR_CLIMB_PORT = 9;
	private final int B_CLIMB_PORT = 10;
	private final int BD_CLIMB_PORT = 11;
	
	// Climb motors
	private TalonSRX fl_climb, fr_climb, b_climb, bd_climb;
	
	
	/**
	 * Constructor.
	 */
	public Climber()
	{
		// Climb motors (aptly named by Scott)
		this.fl_climb = new TalonSRX(FL_CLIMB_PORT);
		this.fr_climb = new TalonSRX(FR_CLIMB_PORT);
		this.b_climb = new TalonSRX(B_CLIMB_PORT);
		this.bd_climb = new TalonSRX(BD_CLIMB_PORT);

		
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
			this.fl_climb.set(ControlMode.PercentOutput, CLIMB_SPD);
		}
		else
		{
			this.fl_climb.set(ControlMode.PercentOutput, 0d);
		}
		// If the robot is retracting the climber and is finished deploying:
		if (climb == MattDupuis.Climber.Climb)
		{
			this.fr_climb.set(ControlMode.PercentOutput, CLIMB_SPD);
		}
		else
		{
			this.fr_climb.set(ControlMode.PercentOutput, 0d);
		}
				// If the robot is retracting the climber and is finished deploying:
		if (climb == MattDupuis.Climber.Climb)
		{
			this.b_climb.set(ControlMode.PercentOutput, CLIMB_SPD);
		}
		else
		{
			this.b_climb.set(ControlMode.PercentOutput, 0d);
		}
		// If the robot is retracting the climber and is finished deploying:
		if (climb == MattDupuis.Climber.Climb)
		{
			this.bd_climb.set(ControlMode.PercentOutput, CLIMB_SPD);
		}
		else
		{
			this.bd_climb.set(ControlMode.PercentOutput, 0d);
		}
	}
}