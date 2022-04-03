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
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

public class Shooter {
	//2 holes above marked one
	//Low shot against hub- 0.25 speed, 1380-1500 rpm
	//High shot from edge of tarmac- 0.5 speed, 3800-4000 rpm

	//3 holes above marked one
	//Low against hub- 0.3 speed, 1650-1750 rpm
	//High from edge of tarmac- 0.525 speed, 2980-3100 rpm
	
	// SPARK MAX
	private CANSparkMax    leadShooter;
	private CANSparkMax    followShooter;
	private DoubleSolenoid shooterBlocker;
	private DoubleSolenoid ballBlocker;

	// SPARK MAX ID's
	private int FOLLOW_SHOOTER_ID  = 19; //follow is left
	private int LEAD_SHOOTER_ID   = 20; //lead is right

	private final int BALL_BLOCKER_DEPLOY_ID     = 1;
	private final int BALL_BLOCKER_RETRACT_ID    = 5;
	private final int SHOOTER_BLOCKER_DEPLOY_ID  = 0;
	private final int SHOOTER_BLOCKER_RETRACT_ID = 7;
	private final int BALL_BLOCKER_PCM_ID        = 1;
	private final int SHOOTER_BLOCKER_PCM_ID     = 2;

	// Encoders
	private RelativeEncoder shooterEncoder;

	// POWER CONSTANTS
	// Find power required to get to target rpm w/o PID Subtract by 0.03
	// Use PID woth mostly I, the P will just give a boost at the start
	public final double OFF_POWER        =  -0.00;
	public final double LOW_SHOT_POWER   =  -0.30; //0.28
	public final double HIGH_SHOT_POWER  =  -0.60; //0.53-comp
	public final double LAUNCH_PAD_POWER =  -0.70; //0.525
	public final double AUTO_RING_POWER  =  -0.60; //0.495

	// RPM CONSTANTS
	public final double OFF_TARGET_RPM              = 6000; //Will always be 0, don't want shooter ready to be true while off

	//Against hub
	public final double LOW_SHOT_TARGET_RPM    = 1800; //Literally no idea

	// 9 feet and 6 inches, from the center of the hub
	public final double HIGH_SHOT_TARGET_RPM   = 3600; //Literally no idea

	// 16 feet and 10 inches, from the center of the hub
	public final double LAUNCH_PAD_TARGET_RPM  = 4200; //Literally no idea

	// 12 feet and 8 inches, from the center of the hub
	public final double AUTO_RING_TARGET_RPM   = 3600; //Literally no idea

	// RPM OFFSET
	private final int RPM_OFFSET = 50;

	// Current Limit Constants
	private static final int SHOOTER_CURRENT_LIMIT = 80;

	// Variables
	public  double                targetVelocity = 0;
	private int                   targetCount    = 0;
	private double                power          = 0;

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
	private PIDController shooterController;

	// Integrator Constants
	private static final double MIN_INTEGRATOR = -0.1; //-0.05
	private static final double MAX_INTEGRATOR =  0.1; // 0.05

	private static final double kP = 0.00008; //0.0000- competition values
	private static final double kI = 0.0006;  //0.0004- competition values
 	private static final double kD = 0.00;

	/****************************************************************************************** 
    *
    *    Constructor
    *   
    ******************************************************************************************/
	public Shooter() {
		// SPARK Max
		leadShooter    = new CANSparkMax(LEAD_SHOOTER_ID, MotorType.kBrushless); //Shooter 2 requires positive power to shoot
		followShooter  = new CANSparkMax(FOLLOW_SHOOTER_ID, MotorType.kBrushless); //Shooter 1 requires negative power to shoot
		ballBlocker    = new DoubleSolenoid(BALL_BLOCKER_PCM_ID   , PneumaticsModuleType.CTREPCM, BALL_BLOCKER_DEPLOY_ID   , BALL_BLOCKER_RETRACT_ID);
		shooterBlocker = new DoubleSolenoid(SHOOTER_BLOCKER_PCM_ID, PneumaticsModuleType.CTREPCM, SHOOTER_BLOCKER_DEPLOY_ID, SHOOTER_BLOCKER_RETRACT_ID);

		// Sets the current limtis for the motors
		leadShooter  .setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
		followShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

		// Sets the mode of the motors (if this works in the code)
		leadShooter  .setIdleMode(CANSparkMax.IdleMode.kCoast);
		followShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
		followShooter.follow(leadShooter, true);

		// Set Shooter related motors to off to Start the Match
		leadShooter.set(0.00);

		// Resets all the pistons
		blockShooter();
		releaseBalls();

		// Encoders
		shooterEncoder = leadShooter.getEncoder();

		// PID Controller
		shooterController = new PIDController(kP, kI, kD);
		shooterController.setIntegratorRange(MIN_INTEGRATOR, MAX_INTEGRATOR);
	}

	/************************************************************
	 * shooterControl()
	 * Designed to be used on new shooter system
	 ***********************************************************/
	public void shooterControl(ShootLocation location) {
		double powerError;
		
		if (location == ShootLocation.HIGH_SHOT) {
			powerError     = shooterController.calculate( getabsRPM(), HIGH_SHOT_TARGET_RPM);
			targetVelocity = HIGH_SHOT_TARGET_RPM;
			power          = HIGH_SHOT_POWER;
		}
		else if (location == ShootLocation.LOW_SHOT) {
			powerError     = shooterController.calculate( getabsRPM(), LOW_SHOT_TARGET_RPM);
			targetVelocity = LOW_SHOT_TARGET_RPM;
			power          = LOW_SHOT_POWER;
		}
		else if (location == ShootLocation.LAUNCH_PAD) {
			powerError     = shooterController.calculate( getabsRPM(), LAUNCH_PAD_TARGET_RPM);
			targetVelocity = LAUNCH_PAD_TARGET_RPM;
			power          = LAUNCH_PAD_POWER;
		}
		else if (location == ShootLocation.AUTO_RING) {
			powerError     = shooterController.calculate( getabsRPM(), AUTO_RING_TARGET_RPM);
			targetVelocity = AUTO_RING_TARGET_RPM;
			power          = AUTO_RING_POWER;
		}
		else {
			powerError      = OFF_POWER;
			targetVelocity  = OFF_TARGET_RPM;
			power           = OFF_POWER;
		}

		power = power + powerError;

		//Displays powers and rpms to smartdashboard
		// SmartDashboard.putNumber("Front power", frontPower);
		// SmartDashboard.putNumber("Front rpm", getabsRPM(FRONT_SHOOTER_ID));
		// SmartDashboard.putNumber("Rear power", rearPower);
		// SmartDashboard.putNumber("Rear rpm", getabsRPM(REAR_SHOOTER_ID));

		leadShooter.set(power);
	}

	/****************************************************************************************** 
    *
    *    shooterReady()
	*    Checks if shooter is ready to fire
    *   
    ******************************************************************************************/
	public boolean shooterReady() {
		//Variables
		double rpm;
		int ON_TARGET_DELAY = 5;

		//Gets rpm values
		rpm  = getabsRPM();

		//Calculates tolerable RPM range
		double lowerLimit  = targetVelocity  - RPM_OFFSET;
		double upperLimit  = targetVelocity  + RPM_OFFSET;

		if ((targetVelocity > 1) && (rpm > lowerLimit) && (rpm < upperLimit)) {
			//System.out.println("Shooter at rpm");
			targetCount ++;
			
			if (targetCount >= ON_TARGET_DELAY) { 
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
	 * Deploys the shooter blocker to stop balls
	 */
	public void blockShooter() {
		shooterBlocker.set(Value.kForward);
	}

	/**
	 * Retracts the shooter blocker to release balls
	 */
	public void openShooter() {
		shooterBlocker.set(Value.kReverse);
	}

	/**
	 * Deploys the ball blocker to stop balls from leaving the robot
	 */
	public void blockBalls() {
		ballBlocker.set(Value.kForward);
	}

	/**
	 * Retracts the ball blocker to allow balls to enter the robot
	 */
	public void releaseBalls() {
		ballBlocker.set(Value.kReverse);
	}

	/**
	 * DEBUG / TEST FUNCTIONS
	 */
	/**
	 * Debug function to disable all motors that are controlled by this class
	 */
	public void disableShooter(){
		leadShooter.set(0);
		blockShooter();
	}

	/**
	 * Deploys all pistons (no ball movement)
	 */
	public void blockAll() {
		blockShooter();
		blockBalls();
	}

	/**
	 * Retracts all pistons (allows ball movement)
	 */
	public void openAll() {
		openShooter();
		releaseBalls();
	}

	/**
	 * Gets the abs RPM of the passed motor
	 * @return absRPM
	 */
	public double getabsRPM() {
		double absRPM = Math.abs(shooterEncoder.getVelocity());
		return absRPM;
	}

	/**
	 * 
	 * @param power
	 */
	public void testShootMotors(double power) {
		//Lead shooter motor 1 needs to be negative to shoot a ball
		//Shooter motor 2 (rear motor) needs to be positive to shoot a ball
		leadShooter.set(-power);
	}

}

//End of the Shooter Class