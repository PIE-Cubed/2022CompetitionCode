package frc.robot;

/**
 * Imports
 */
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
	//2 holes above marked one
	//Low shot against hub- 0.25 speed, 1380-1500 rpm
	//High shot from edge of tarmac- 0.5 speed, 3800-4000 rpm

	//3 holes above marked one
	//Low against hub- 0.3 speed, 1650-1750 rpm
	//High from edge of tarmac- 0.525 speed, 2980-3100 rpm
	
	// SPARK MAX
	private CANSparkMax    frontShooter;
	private CANSparkMax    rearShooter;
	private DoubleSolenoid feeder;

	// SPARK MAX ID's
	private int FRONT_SHOOTER_ID  = 19; //front
	private int REAR_SHOOTER_ID   = 20; //right-->rear

	private int FEEDER_DEPLOY_ID  = 0;
	private int FEEDER_RETRACT_ID = 7;
	private int PCM_CAN_ID        = 2;

	// Encoders
	private RelativeEncoder frontShooterEncoder;
	private RelativeEncoder rearShooterEncoder;

	// POWER CONSTANTS
	public final double OFF_POWER  = 0.00;

	public final double LOW_SHOT_REAR_POWER    =  0.31; //0.28
	public final double LOW_SHOT_FRONT_POWER   = -0.27; //-0.27

	public final double HIGH_SHOT_REAR_POWER   =  0.53; //0.465
	public final double HIGH_SHOT_FRONT_POWER  = -0.51; //0.445

	public final double LAUNCH_PAD_REAR_POWER  =  0.56; //0.525
	public final double LAUNCH_PAD_FRONT_POWER = -0.55; //0.525

	public final double AUTO_RING_REAR_POWER   =  0.53; //0.495
	public final double AUTO_RING_FRONT_POWER  = -0.51; //0.475

	// RPM CONSTANTS
	public final double OFF_TARGET_RPM              = 0; //Will always be 0

	// Against the board
	public final double LOW_SHOT_REAR_TARGET_RPM    = 1650; //1650
	public final double LOW_SHOT_FRONT_TARGET_RPM   = 1650; //1650

	// 9 feet and 6 inches, from the center of the hub
	public final double HIGH_SHOT_REAR_TARGET_RPM   = 2800; //2700
	public final double HIGH_SHOT_FRONT_TARGET_RPM  = 2800; //2700

	// 16 feet and 10 inches, from the center of the hub
	public final double LAUNCH_PAD_REAR_TARGET_RPM  = 3150; //2900
	public final double LAUNCH_PAD_FRONT_TARGET_RPM = 3150; //2900

	// 12 feet and 8 inches, from the center of the hub
	public final double AUTO_RING_REAR_TARGET_RPM   = 2800; //2700
	public final double AUTO_RING_FRONT_TARGET_RPM  = 2800; //2700

	// RPM OFFSET
	private final int RPM_OFFSET = 40;

	// Current Limit Constants
	private static final int SHOOTER_CURRENT_LIMIT = 80;

	// Variables
	public  double                frontTargetVelocity;
	public  double                rearTargetVelocity;
	private int                   targetCount        = 0;
	private double                frontPower         = 0;
	private double                rearPower          = 0;

	public static enum ShootLocation {
		HIGH_SHOT,
		LOW_SHOT,
		LAUNCH_PAD,
		AUTO_RING,
		OFF;
	}

	public static enum BallFeederDirection {
		FORWARD,
		REVERSE,
		OFF;
	}

	// Shooter PID Controller
	private PIDController frontShooterController;
	private PIDController rearShooterController;

	// Integrator Constants
	private static final double MIN_INTEGRATOR = -0.1; //-0.05
	private static final double MAX_INTEGRATOR =  0.1; //0.05

	// P, I, D constants
	private static final double kP = 0.0000; //0.00010
	private static final double kI = 0.0004; //0.0001
	private static final double kD = 0.00;


	/****************************************************************************************** 
    *
    *    Constructor
    *   
    ******************************************************************************************/
	public Shooter() {
		// SPARK Max
		frontShooter  = new CANSparkMax(FRONT_SHOOTER_ID, MotorType.kBrushless); //Shooter 1 requires negative power to shoot
		rearShooter   = new CANSparkMax(REAR_SHOOTER_ID, MotorType.kBrushless); //Shooter 2 requires positive power to shoot
		feeder        = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, FEEDER_DEPLOY_ID, FEEDER_RETRACT_ID);

		// Sets the current limtis for the motors
		frontShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
		rearShooter .setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

		// Sets the mode of the motors (if this works in the code)
		frontShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rearShooter .setIdleMode(CANSparkMax.IdleMode.kCoast);
		
		// Set Shooter related motors to off to Start the Match
		frontShooter.set(0.0);
		rearShooter .set(0.0);
		retractFeeder();

		// Encoders
		frontShooterEncoder = frontShooter.getEncoder();
		rearShooterEncoder  = rearShooter.getEncoder();

		// PID Controller
		frontShooterController = new PIDController(kP, kI, kD);
		rearShooterController  = new PIDController(kP, kI, kD);
		frontShooterController.setIntegratorRange(MIN_INTEGRATOR, MAX_INTEGRATOR);

	}


	/****************************************************************************************** 
    *
    *    autoShooterControl()
	*    Uses PID to get shooter to appropriate speed for given shot
    *   
	******************************************************************************************/
	public void shooterControl(ShootLocation location) {
		double frontPowerError;
		double rearPowerError;
		
		if (location == ShootLocation.HIGH_SHOT) {
			frontPowerError     = frontShooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), HIGH_SHOT_FRONT_TARGET_RPM);
			rearPowerError      = rearShooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , HIGH_SHOT_REAR_TARGET_RPM);
			frontTargetVelocity = HIGH_SHOT_FRONT_TARGET_RPM;
			rearTargetVelocity  = HIGH_SHOT_REAR_TARGET_RPM;
			frontPower          = HIGH_SHOT_FRONT_POWER;
			rearPower           = HIGH_SHOT_REAR_POWER;
		}
		else if (location == ShootLocation.LOW_SHOT) {
			frontPowerError     = frontShooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), LOW_SHOT_FRONT_TARGET_RPM);
			rearPowerError      = rearShooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , LOW_SHOT_REAR_TARGET_RPM);
			frontTargetVelocity = LOW_SHOT_FRONT_TARGET_RPM;
			rearTargetVelocity  = LOW_SHOT_REAR_TARGET_RPM;
			frontPower          = LOW_SHOT_FRONT_POWER;
			rearPower           = LOW_SHOT_REAR_POWER;
		}
		else if (location == ShootLocation.LAUNCH_PAD) {
			frontPowerError     = frontShooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), LAUNCH_PAD_FRONT_TARGET_RPM);
			rearPowerError      = rearShooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , LAUNCH_PAD_REAR_TARGET_RPM);
			frontTargetVelocity = LAUNCH_PAD_FRONT_TARGET_RPM;
			rearTargetVelocity  = LAUNCH_PAD_REAR_TARGET_RPM;
			frontPower          = LAUNCH_PAD_FRONT_POWER;
			rearPower           = LAUNCH_PAD_REAR_POWER;
		}
		else if (location == ShootLocation.AUTO_RING) {
			frontPowerError     = frontShooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), AUTO_RING_FRONT_TARGET_RPM);
			rearPowerError      = rearShooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , AUTO_RING_REAR_TARGET_RPM);
			frontTargetVelocity = AUTO_RING_FRONT_TARGET_RPM;
			rearTargetVelocity  = AUTO_RING_REAR_TARGET_RPM;
			frontPower          = AUTO_RING_FRONT_POWER;
			rearPower           = AUTO_RING_REAR_POWER;
		}
		else {
			frontPowerError     = OFF_POWER;
			rearPowerError      = OFF_POWER;
			frontTargetVelocity = OFF_TARGET_RPM;
			rearTargetVelocity  = OFF_TARGET_RPM;
			frontPower          = OFF_POWER;
			rearPower           = OFF_POWER;
		}

		//Resets the I value of PID until we are at 80% speed to prevent 
		//I value from growing out of control at start
		if (getabsRPM(FRONT_SHOOTER_ID) < frontTargetVelocity * 0.8) {
			frontShooterController.reset();
			//Low shot needs more of a kick to get up to speed
			if (location == ShootLocation.LOW_SHOT) {
				frontPower *= 1.25;
			}
			else {
				frontPower *= 1.15;
			}
		}
		else {
			frontPower = frontPower - frontPowerError;
			frontPower = MathUtil.clamp(frontPower, -1.0, 0.0);
		}

		if (getabsRPM(REAR_SHOOTER_ID) < rearTargetVelocity * 0.8) {
			rearShooterController.reset();
			if (location == ShootLocation.LOW_SHOT) {
				rearPower *= 1.25;
			}
			else {
				rearPower *= 1.15;
			}
		}
		else {
			rearPower  = rearPower + rearPowerError;
			rearPower  = MathUtil.clamp(rearPower, 0.0, 1.0);
		}

		//Displays powers and rpms to smartdashboard
		// SmartDashboard.putNumber("Front power", frontPower);
		// SmartDashboard.putNumber("Front rpm", getabsRPM(FRONT_SHOOTER_ID));
		// SmartDashboard.putNumber("Rear power", rearPower);
		// SmartDashboard.putNumber("Rear rpm", getabsRPM(REAR_SHOOTER_ID));

		frontShooter.set(frontPower);
		rearShooter.set(rearPower);
	}

	/****************************************************************************************** 
    *
    *    shooterReady()
	*    Checks if shooter is ready to fire
    *   
    ******************************************************************************************/
	public boolean shooterReady() {
		//Variables
		double rearRpm;
		double frontRpm;
		final int ON_TRAGET_DELAY = 10;

		//Gets rpm values
		rearRpm  = getabsRPM(REAR_SHOOTER_ID);
		frontRpm = getabsRPM(FRONT_SHOOTER_ID);

		//Calculates tolerable RPM range
		double rearLowerLimit  = rearTargetVelocity  - RPM_OFFSET;
		double rearUpperLimit  = rearTargetVelocity  + RPM_OFFSET;
		double frontLowerLimit = frontTargetVelocity - RPM_OFFSET;
		double frontUpperLimit = frontTargetVelocity + RPM_OFFSET;

		if ((rearTargetVelocity > 1)    && (frontTargetVelocity > 1) && 
			(rearRpm  > rearLowerLimit  && rearRpm  < rearUpperLimit) && 
			(frontRpm > frontLowerLimit && frontRpm < frontUpperLimit))  {
			//System.out.println("Shooter at rpm");
			targetCount ++;
			
			if (targetCount >= ON_TRAGET_DELAY) { 
				//System.out.println("Shooter ready");
				return true;
			}
			else {
				return false;
			}
		}
		else {
			// Resets targetCount
			targetCount = 0;
			return false;
		}
	}
	
	/**
	 * Raises the ball feeder
	 */
	public void deployFeeder() {
		feeder.set(Value.kForward);
	}

	/**
	 * Lowers the ball feeder
	 */
	public void retractFeeder() {
		feeder.set(Value.kReverse);
	}

	/**
	 * DEBUG / TEST FUNCTIONS
	 */
	/**
	 * Debug function to disable all motors that are controlled by this class
	 */
	public void disableShooter(){
		frontShooter.set(0);
		rearShooter.set(0);
		retractFeeder();
	}

	/**
	 * Function to display all the different motor RPM's
	 */
	public void displayRPMValues() {
		System.out.println("Shooter 1 RPM: " + shooter1RPM());
		System.out.println("Shooter 2 RPM: " + shooter2RPM());
	}

	private double shooter1RPM() {
		double rpm;
		rpm = getabsRPM(FRONT_SHOOTER_ID);
		return rpm;
	}

	private double shooter2RPM() {
		double rpm;
		rpm = getabsRPM(REAR_SHOOTER_ID);
		return rpm;
	}

	/**
	 * Gets the abs RPM of the passed motor
	 * @return absRPM
	 */
	public double getabsRPM(int MOTOR_CAN_ID) {
		double rpm;
		double absRPM;

		if (MOTOR_CAN_ID == FRONT_SHOOTER_ID) {
			rpm = frontShooterEncoder.getVelocity();
		}
		else if (MOTOR_CAN_ID == REAR_SHOOTER_ID) {
			rpm = rearShooterEncoder.getVelocity();
		}
		else {
			//It should never come to this case
			rpm = 0;
		}

		absRPM = Math.abs(rpm);
		return absRPM;
	}

	/**
	 * 
	 * @param power
	 */
	public void testShootMotors(double power) {
		//Shooter motor 1 (front motor) needs to be negative to shoot a ball
		//Shooter motor 2 (rear motor) needs to be positive to shoot a ball
		frontShooter.set(-power);
		rearShooter.set(power);
		System.out.println("Front RPM: " + getabsRPM(FRONT_SHOOTER_ID) + " Rear RPM: " + getabsRPM(REAR_SHOOTER_ID));
	}

} //End of the Shooter Class