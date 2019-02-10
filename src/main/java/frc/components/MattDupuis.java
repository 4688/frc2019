// JACOB CAZABON <3 SAINTS BOT 2018

package frc.components;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Reads and processes input from the robot driver (after whom this class is
 * honourably named) through both the main Xbox controller and the button board.
 * 
 * @author Jacob
 */
public class MattDupuis
{
	// Xbox controller and button board USB ports
	private final int DRIVER_USB = 0;
	private final int BOARD_USB = 1;
	
	// Driver axis indices
	private final int FORWARD_DAXIS = 1;
	private final int TURN_DAXIS = 4;
	private final int TURBO_DAXIS = 2;
	private final int INTAKE_DAXIS = 3;
	
	// Driver button indices
	private final int EJECT_DBTN = 2;
	private final int RAISE_DBTN = 6;
	private final int LOWER_DBTN = 5;
	
	// Button board indices
	private final int DEPLOY_BBTN = 12;
	private final int CLIMB_BBTN = 11;
	private final int LIFTRAISE_BBTN = 8;
	private final int LIFTLOWER_BBTN = 10;
	private final int TILTUP_BBTN = 7;
	private final int TILTDN_BBNT = 9;
	
	// Length of rumble, in 1/50ths of a second
	private final int RUMBLE_TIME = 15;
	
	// Xbox controller and button board
	private Joystick driver, board;
	
	// Rumble timers
	private int rumbleL, rumbleR;
	
	/**
	 * Constructor.
	 */
	public MattDupuis()
	{
		// Initialize joysticks
		this.driver = new Joystick(DRIVER_USB);
		this.board = new Joystick(BOARD_USB);
		
		// Initialize rumble timers
		this.rumbleL = 0;
		this.rumbleR = 0;
	}
	
	/**
	 * Updates the rumble pulse of the controller based on whether a switch is
	 * pressed. The controller should only pulse each time the switch is pressed
	 * and only once until it is released.
	 * 
	 * @param pressed True if the switch is pressed, false otherwise
	 */
	public void rumbleRight(boolean pressed)
	{
		// Activate rumble if switch is pressed
		if (false && ((pressed && this.rumbleR == 0) || this.rumbleR > 0))
		{
			double rumble = Math.sqrt(RUMBLE_TIME - this.rumbleR) / Math.sqrt(RUMBLE_TIME);
			if (Double.isNaN(rumble) && !pressed)
			{
				this.rumbleR = 0;
			}
			else
			{
				this.rumbleR += 1;
			}
			this.driver.setRumble(RumbleType.kRightRumble, rumble);
		}
		else
		{
			this.rumbleR = 0;
		}
	}
	
	/**
	 * Updates the rumble pulse of the controller based on whether a switch is
	 * pressed. The controller should only pulse each time the switch is pressed
	 * and only once until it is released.
	 * 
	 * @param pressed True if the switch is pressed, false otherwise
	 */
	public void rumbleLeft(boolean pressed)
	{
		// Activate rumble if switch is pressed 
		if (false && ((pressed && this.rumbleL == 0) || this.rumbleL > 0))
		{
			double rumble = Math.sqrt(RUMBLE_TIME - this.rumbleL) / Math.sqrt(RUMBLE_TIME);
			if (Double.isNaN(rumble) && !pressed)
			{
				this.rumbleL = 0;
			}
			else
			{
				this.rumbleL += 1;
			}
			this.driver.setRumble(RumbleType.kLeftRumble, rumble);
		}
		else
		{
			this.rumbleL = 0;
		}
	}
	
	/**
	 * Gets the inverted forward axis between -1 and 1; 1 means 100% throttle
	 * forward, 0 means no forward motion, and -1 means 100% throttle backwards.
	 * The value is squared whilst preserving the sign to allow for finer
	 * control at mid to low speeds.
	 * 
	 * @return The inverted forward axis, between -1 and 1
	 */
	public double getForward()
	{
		double fwd = -this.driver.getRawAxis(FORWARD_DAXIS);
		return fwd * Math.abs(fwd);
	}
	
	/**
	 * Gets the turning axis between -1 and 1; 1 means a full speed left turn, 0
	 * means no turning motion, and -1 means a full speed right turn.
	 * The value is squared whilst preserving the sign to allow for finer
	 * control at mid to low speeds.
	 * 
	 * @return The turning axis, between -1 and 1
	 */
	public double getTurn()
	{
		double turn = this.driver.getRawAxis(TURN_DAXIS);
		return turn * Math.abs(turn);
	}
	
	/**
	 * Gets the turbo axis between 0 and 1; 1 means full turbo and 0 means no
	 * turbo (regular speed).
	 * 
	 * @return The turbo axis, between 0 and 1
	 */
	public double getTurbo()
	{
		return this.driver.getRawAxis(TURBO_DAXIS);
	}
	
	/**
	 * Gets the intake axis between -1 and 1; 1 means full eject speed, 0 means
	 * no motion, and -1 means full intake speed.
	 * 
	 * @return The intake axis, between -1 and 1
	 */
	public double getIntake()
	{
		boolean reverse = this.driver.getRawButton(EJECT_DBTN);
		return this.driver.getRawAxis(INTAKE_DAXIS) * (reverse ? -1d : 1d);
	}
	
	/**
	 * Enumeration of possible hugger tilt actions.
	 */
	public static enum Tilt
	{
		None, Up, Down
	}
	
	/**
	 * Returns what action is currently being performed with regards to the
	 * hugger tilt.
	 * 
	 * @return Whether the hugger should tilt up, down, or not at all
	 */
	public Tilt getTilt()
	{
		// Get DPad value
		int dpad = this.driver.getPOV();
		
		// If pressing DPad up, tilt up
		if (dpad == 0) return Tilt.Up;
		
		// If pressing DPad down, tilt down
		else if (dpad == 180) return Tilt.Down;
				
		// If pressing anywhere else, do not tilt
		return Tilt.None;
	}
	
	/**
	 * Enumeration of possible lift actions.
	 */
	public static enum Lift
	{
		None, Raise, Lower
	}
	
	/**
	 * Returns what action is currently being performed by the lift.
	 * 
	 * @return Whether the lift is being raised, being lowered, or not moving
	 */
	public Lift getLift()
	{
		// Check raise/lower buttons
		boolean raise = this.driver.getRawButton(RAISE_DBTN);
		boolean lower = this.driver.getRawButton(LOWER_DBTN);
		
		// If only one of the buttons are being pressed, return the action
		if (raise && !lower) return Lift.Raise;
		if (lower && !raise) return Lift.Lower;
		
		// Otherwise, do nothing
		return Lift.None;
	}
	
	/**
	 * Enumeration of possible climber actions.
	 */
	public static enum Climber
	{
		None, Deploy, Climb
	}
	
	/**
	 * Returns what action is currently being performed by the climber.
	 * 
	 * @return Whether the climber is being deployed, climbing, or not moving
	 */
	public Climber getClimber()
	{
		// Should be good now? ;)
		if (true || DriverStation.getInstance().getMatchTime() <= 30d)
		{
			if (this.board.getRawButton(DEPLOY_BBTN)) return Climber.Deploy;
			if (this.board.getRawButton(CLIMB_BBTN)) return Climber.Climb;
		}
		return Climber.None;
	}
	
	/**
	 * Returns whether the user button on the RIO is pressed.
	 * 
	 * @return True if the RIO user button is pressed, false otherwise
	 */
	public boolean isZeroingSensors()
	{
		return RobotController.getUserButton();
	}
}