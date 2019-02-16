// JACOB CAZABON <3 SAINTS BOT 2018

package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot{

	private final int JOYSTICK_USB = 0;
	private MattDupuis matt;
	private DriveTrain driveTrain;
	private Grabber grabber;
	private Lift lift;
	private Climber climber;
	private Wrist wrist;
	
	public void robotInit()
	{
		this.matt = new MattDupuis(JOYSTICK_USB);
		this.driveTrain = new DriveTrain();
		this.grabber = new Grabber();
		this.lift = new Lift();
		this.climber = new Climber();
		this.wrist = new Wrist();
	}
	
	public void robotPeriodic()
	{

		if (RobotController.getUserButton())
		{
			this.driveTrain.zeroSensors();
			this.lift.zeroSensors();
		}
	}
	
	public void disabledInit()
	{
	}
	
	public void autonomousInit()
	{
	}
	
	public void teleopInit()
	{
	}
	
	public void teleopPeriodic()
	{

		SmartDashboard.putBoolean("Top", lift.highLim.get());
		SmartDashboard.putBoolean("Bottom", lift.lowLim.get());
		SmartDashboard.putBoolean("HighLim", wrist.highLim.get());
		SmartDashboard.putNumber("Lift Encoder", lift.liftEnc.getRaw());
		this.driveTrain.control(this.matt, this.lift.getTravel());
		this.grabber.control(this.matt);
   		this.lift.control(this.matt);
		this.climber.control(this.matt);
		this.wrist.control(this.matt);
		
		if (DriverStation.getInstance().getMatchTime() <= 30d)
		{
			this.climber.control(this.matt);
		}
	}
	
	public void testInit()
	{
	}
	private static class MattDupuis
	{
    private Joystick joystick;
    private Joystick ds_button;
		
		public MattDupuis(int joystickUsb)
		{
      this.joystick = new Joystick(joystickUsb);
      this.ds_button = new Joystick(1);
		}
		
		public double getForward()
		{
			double fwd = -this.joystick.getRawAxis(1);
			return fwd * Math.abs(fwd);
		}
		
		public double getTurn()
		{
			double turn = this.joystick.getRawAxis(4);
			return turn * Math.abs(turn);
		}
		
		public double getTurbo()
		{
			return 1d + this.joystick.getRawAxis(3) * 0.5;
		}
		
		public double getIntake()
		{
			double inVal = (this.joystick.getRawAxis(3) * -0.5);
			if(this.joystick.getRawButton(2)){
				inVal = 1d;
			}
			return(inVal);
		}

		public double getWrist()
	{
		// Get DPad value
		int dpad = this.joystick.getPOV();
		
		if (dpad == 0) return 1;
		
		// If pressing DPad down, tilt down
		else if (dpad == 180) return -1;
				
		else return 0;
	}
		
		public double getLift()
		{
			boolean up = this.joystick.getRawButton(6);
			boolean dn = this.joystick.getRawButton(5);
			if (up && !dn)
			{
				return 1d;
			}
			else if (!up && dn)
			{
				return -0.2d;
			}
			else
			{
				return 0d;
			}
	}
	
	public boolean getPistons(){
		return(this.joystick.getRawButton(1));
	}

    public double all3()
		{
			boolean all3up = this.ds_button.getRawButton(2);
			boolean all3down = this.ds_button.getRawButton(1);
			if (all3up && !all3down)
			{
				return 1d;
			}
			else if (!all3up && all3down)
			{
				return -1d;
			}
			else
			{
				return 0d;
			}
		}
		
		public double front2()
		{
			boolean front2up = this.ds_button.getRawButton(3);
			boolean front2down = this.ds_button.getRawButton(4);
			if (front2up && !front2down)
			{
				return 1d;
			}
			else if (!front2up && front2down)
			{
				return -1d;
			}
			else
			{
				return 0d;
			}
		}

		public double back_drive()
		{
			boolean back_drive = this.ds_button.getRawButton(5);
			if (back_drive)
			{
				return 1d;
			}
			else
			{
				return 0d;
			}
		}

		public double back_legs()
		{
			boolean back_leg_up = this.ds_button.getRawButton(6);
			boolean back_leg_down = this.ds_button.getRawButton(7);
			if (back_leg_up && !back_leg_down)
			{
				return 1d;
			}
			else if (!back_leg_up && back_leg_down)
			{
				return -1d;
			}
			else
			{
				return 0d;
			}
		}
	}
	
	private static class DriveTrain
	{
		private static final double DRIVE_FACTOR = 0.5;

		private static final double k_max = 0.5; // Maximum speed at max lift height
		private static final double E_max = -7d; // Encoder value at maximum height; needs testing
		
    private TalonSRX lfm, rfm;
    private VictorSPX lrm, rrm;
		
		public DriveTrain()
		{
			this.lfm = new TalonSRX(1);
			this.lrm = new VictorSPX(3);
			this.rfm = new TalonSRX(2);
			this.rrm = new VictorSPX(4);
			
			this.lrm.follow(this.lfm);
			this.rrm.follow(this.rfm);
		}
		
		public void control(MattDupuis matt, double liftTravel)
		{
			double y = matt.getForward();
			double x = matt.getTurn();
			double l = 0d, r = 0d;
			double d = DRIVE_FACTOR;
			double t = matt.getTurbo();
			double e = 1d; // Math.min(Math.max(0, -(k_max / Math.pow(E_max,2))*Math.pow(liftTravel,2) + 1), 1);
			if (Math.abs(x) < 0.04)
			{
				l = y;
				r = -y;
			}
			else if (Math.abs(y) < 0.04)
			{
				l = x;
				r = x;
			}
			else if (Math.abs(x) >= 0.04)
			{
				l = x + y;
				r = x - y;
			}
			this.setLSpd(l * d * t * e);
			this.setRSpd(r * d * t * e);
		}
		
		public void setLSpd(double spd)
		{
			if (Math.abs(spd) > 0.04)
			{
				this.lfm.set(ControlMode.PercentOutput, spd);
			}
			else
			{
				this.lfm.set(ControlMode.PercentOutput, 0d);
			}
		}
		
		public void setRSpd(double spd)
		{
			if (Math.abs(spd) > 0.04)
			{
				this.rfm.set(ControlMode.PercentOutput, spd);
			}
			else
			{
				this.rfm.set(ControlMode.PercentOutput, 0d);
			}
		}
		
		public void zeroSensors()
		{
			this.lfm.getSensorCollection().setQuadraturePosition(0, 0);
		}
		
		public double getRevs()
		{
			return -this.lfm.getSensorCollection().getQuadraturePosition() / 4096d;
		}
	}

	private static class Grabber
	{
		private VictorSPX grabber;
		private Solenoid pistons;
		private Compressor compressor;
		
		public Grabber()
		{
			this.grabber = new VictorSPX(7);
			this.pistons = new Solenoid(1, 0);
			this.compressor = new Compressor(1);
			compressor.setClosedLoopControl(true);
		}
		
		public void control(MattDupuis matt)
		{
			this.grabber.set(ControlMode.PercentOutput, matt.getIntake());
			this.pistons.set(matt.getPistons());
		}
	}
	
	private static class Lift
	{
		private DigitalInput lowLim, highLim;
		private Spark lifty;
		private Encoder liftEnc;
		
		public Lift()
		{
			this.lowLim = new DigitalInput(2);
			this.highLim = new DigitalInput(3);
			this.lifty = new Spark(5);
			this.liftEnc = new Encoder(6, 7);
			this.liftEnc.setDistancePerPulse(1d / 2048d); // May need to adjust resolution
		}
		
		public void control(MattDupuis matt){
			double lift = matt.getLift();
			if ((lift < 0d && this.lowLim.get()) || (lift > 0d && this.highLim.get()))
			{
				this.lifty.set(lift);
			}
			else  
			{
				if (!this.lowLim.get()){
					this.lifty.set(0d);	
				}else{
					this.lifty.set(0.4d);	
				}		
			}
		}
		
		public void zeroSensors()
		{
			this.liftEnc.reset();
		}
		
		public double getTravel()
		{
			return -this.liftEnc.getDistance();
		}
	}

	public static class Wrist{

		private VictorSPX wrist;
		private DigitalInput lowLim, highLim;

		public Wrist()
		{
			this.wrist = new VictorSPX(6);
			this.highLim = new DigitalInput(9);
			this.lowLim = new DigitalInput(4);
		}
		public void control(MattDupuis matt){
			double spd = matt.getWrist();
			if (!this.highLim.get() || spd < 0){
				this.wrist.set(ControlMode.PercentOutput, spd);
			}else{
				this.wrist.set(ControlMode.PercentOutput, 0d);
			}
		}
	}

	// (this.lowLim.get() || spd > 0) && 
	
	
  public class Climber{
		
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
		
		public void control(MattDupuis matt)
			{
				double all3value = matt.all3();
				double front2value = matt.front2();
				double back_drive_value = matt.back_drive();
				double back_legs_value = matt.back_legs();
				double back_percent = 1d;
				double front_left_percent = 1d;
				double front_right_percent = 1d;

				if (all3value != 0)
				{
					this.fl_climb.set(ControlMode.PercentOutput, (all3value * front_left_percent));
					this.fr_climb.set(ControlMode.PercentOutput, (all3value * front_right_percent));
					this.b_climb.set(ControlMode.PercentOutput, (all3value * back_percent));
				}
				else
				{
					this.fl_climb.set(ControlMode.PercentOutput, (front2value * front_left_percent));
					this.fr_climb.set(ControlMode.PercentOutput, (front2value * front_right_percent));
					this.b_climb.set(ControlMode.PercentOutput, (back_legs_value * back_percent));
				}
				
				this.bd_climb.set(ControlMode.PercentOutput, back_drive_value);

			}
		}
  }