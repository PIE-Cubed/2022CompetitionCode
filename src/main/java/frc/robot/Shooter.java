package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
	
	// SPARK MAX
	private CANSparkMax leftShooter;
	private CANSparkMax rightShooter;

	// SPARK MAX ID's
	private int LEFT_SHOOTER_ID  = 19;
	private int RIGHT_SHOOTER_ID = 21;

	// Encoders
	private RelativeEncoder leftShooterEncoder;
	private RelativeEncoder rightShooterEncoder;

	// POWER CONSTANTS
	public final double OFF_POWER  = 0.00;
	public final double HIGH_SHOT_RIGHT_POWER = -0.70;
	public final double HIGH_SHOT_LEFT_POWER = -0.70;
	public final double LOW_SHOT_RIGHT_POWER  = -0.40;
	public final double LOW_SHOT_LEFT_POWER  = -0.40;

	// RPM CONSTANTS
	public final double OFF_TARGET_RPM             = 0;
	public final double HIGH_SHOT_RIGHT_TARGET_RPM = 4000;
	public final double HIGH_SHOT_LEFT_TARGET_RPM  = 4000;
	public final double LOW_SHOT_RIGHT_TARGET_RPM  = 2000;
	public final double LOW_SHOT_LEFT_TARGET_RPM   = 2000;

	// Current Limit Constants
	private static final int SHOOTER_CURRENT_LIMIT = 80;

	// Variables
	public  double                leftTargetVelocity;
	public  double                rightTargetVelocity;
	private int                   targetCount        = 0;
	Shooter.ShootLocation         startPosition      = Shooter.ShootLocation.OFF;

	public static enum ShootLocation {
		HIGH_SHOT,
		LOW_SHOT,
		OFF;
	}

	public static enum BallFeederDirection {
		FORWARD,
		REVERSE,
		OFF;
	}


	// Shooter PID Controller
	private PIDController shooterController;

	private static final double kP = 0.0001;
	private static final double kI = 0.00;
	private static final double kD = 0.00;


	/****************************************************************************************** 
    *
    *    Constructor
    *   
    ******************************************************************************************/
	public Shooter() {
		// SPARK Max
		leftShooter    = new CANSparkMax(LEFT_SHOOTER_ID, MotorType.kBrushless); //Shooter 1 requires negative power to shoot
		rightShooter   = new CANSparkMax(RIGHT_SHOOTER_ID, MotorType.kBrushless); //Shooter 2 requires positive power to shoot

		// Sets the current limtis for the motors
		leftShooter .setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
		rightShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

		// Sets the mode of the motors (if this works in the code)
		leftShooter. setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
		
		// Set Shooter related motors to off to Start the Match
		leftShooter .set(0.0);
		rightShooter.set(0.0);

		// Encoders
		leftShooterEncoder  = leftShooter.getEncoder();
		rightShooterEncoder = rightShooter.getEncoder();

		// PID Controller
		shooterController = new PIDController(kP, kI, kD);
	}


	/****************************************************************************************** 
    *
    *    autoShooterControl()
	*    Uses PID to get shooter to appropriate speed for given shot
    *   
	******************************************************************************************/
	/*
	public void autoShooterControl(ShootLocation location) {
		shotLocation = location;
		double  powerError;
		double  power;

		if (location == ShootLocation.OFF) {
			powerError     = OFF_POWER;
			targetVelocity = OFF_TARGET_RPM;
			targetPower    = OFF_POWER;
		}
		else if ( ((location == ShootLocation.TEN_FOOT) || (location == ShootLocation.TRENCH)) || (location == ShootLocation.HAIL_MARY) ) {
			powerError     = shooterController.calculate( getabsRPM(LEFT_SHOOTER_ID), SHOOT_TARGET_RPM);
			targetVelocity = SHOOT_TARGET_RPM;
			targetPower    = SHOOT_POWER;
		}
		else {
			powerError     = OFF_POWER;
			targetVelocity = OFF_TARGET_RPM;
			targetPower    = OFF_POWER;
		}

		power = targetPower + powerError;
		power = MathUtil.clamp(power, 0.0, 1.0);
		
		System.out.println("power:" + power);
		System.out.println("rpm:" + getabsRPM(LEFT_SHOOTER_ID));
		
		SmartDashboard.putNumber("power", power);
		SmartDashboard.putNumber("rpm", getabsRPM(LEFT_SHOOTER_ID));

		rightShooter.set(power);
	}*/


	/****************************************************************************************** 
    *
    *    manualShooterControl()
	*    Gets shooter to appropriate speed without PID
    *   
    ******************************************************************************************/
	public void manualShooterControl(ShootLocation location) {
											
	if (location == ShootLocation.OFF) {
			rightShooter.set(OFF_POWER);
			leftShooter.set(OFF_POWER);
			rightTargetVelocity = OFF_TARGET_RPM;
			leftTargetVelocity  = OFF_TARGET_RPM;
		}
		else if (location == ShootLocation.HIGH_SHOT) {
			rightShooter.set(HIGH_SHOT_RIGHT_POWER);
			leftShooter.set(HIGH_SHOT_LEFT_POWER);
			rightTargetVelocity = HIGH_SHOT_RIGHT_TARGET_RPM;
			leftTargetVelocity  = HIGH_SHOT_LEFT_TARGET_RPM;
		}
		else if (location == ShootLocation.LOW_SHOT) {
			rightShooter.set(LOW_SHOT_RIGHT_POWER);
			leftShooter.set(LOW_SHOT_LEFT_POWER);
			rightTargetVelocity = LOW_SHOT_RIGHT_TARGET_RPM;
			leftTargetVelocity  = LOW_SHOT_LEFT_TARGET_RPM;
		}
		else {
			rightShooter.set(OFF_POWER);
			leftShooter.set(OFF_POWER);
			rightTargetVelocity = OFF_TARGET_RPM;
			leftTargetVelocity  = OFF_TARGET_RPM;
		}
	}


	/****************************************************************************************** 
    *
    *    shooterReady()
	*    Checks if shooter is ready to fire
    *   
    ******************************************************************************************/
	public boolean shooterReady() {
		double rightRpm;
		double leftRpm;
		rightRpm = getabsRPM(RIGHT_SHOOTER_ID);
		leftRpm  = getabsRPM(LEFT_SHOOTER_ID);
				
		if ( (rightRpm > rightTargetVelocity) && (leftRpm > leftTargetVelocity))  {
			targetCount ++;
			
			if(targetCount >= 5) { 
				return true;
			}
			else {
				return false;
			}
		}
		else {
			targetCount = 0;
			return false;
		}
	}



	/**
	 * DEBUG / TEST FUNCTIONS
	 */
	
	 /**
	 * A debug function for the shooter
	 * @param power
	 */
	public void testShoooter(double power) {
		leftShooter.set(power);
		rightShooter.set(power);

		System.out.println("Power: " + power + " RPM: " + getabsRPM(LEFT_SHOOTER_ID));
	}

	/**
	 * Prints the speed of the wheel
	 */
	public void printSpeed() {
		double π = Math.PI;
		double wheel_size = 6;                                                 // Wheel diameter Inches 

		double RPM = (getabsRPM(LEFT_SHOOTER_ID) + getabsRPM(RIGHT_SHOOTER_ID) ) / 2; // Rotations per minute average
		double RPH = RPM / 60;                                                 // Rotations per hour
		
		double circumferenceInches = wheel_size * π;                           // Circumference in Inches
		double circumferenceFeet = circumferenceInches / 12;                   // Circumference in Feet
		double circumferenceMiles = circumferenceFeet / 5280;                  // Circumference in Miles

		double MPH = RPH * circumferenceMiles;                                 // Miles Per Hour

		if (RPM > 0) { 
			//System.out.println("MPH: " + MPH);
			if (MPH != 0) {
				System.out.println("RPM 1: " + getabsRPM(LEFT_SHOOTER_ID));
				System.out.println("RPM 2: " + getabsRPM(RIGHT_SHOOTER_ID));
			}
		}
	}

	public void enableShooterFullPower() {
		leftShooter.set(1.00);
		rightShooter.set(1.00);

		System.out.println("RPM 1: " + getabsRPM(LEFT_SHOOTER_ID));
		System.out.println("RPM 2: " + getabsRPM(RIGHT_SHOOTER_ID));
	}

	/**
	 * Debug function to enable shooting motors 
	 */
	public void testShooter(double power) {
		rightShooter.set(-1 * .25);
		leftShooter.set(1.00);
	}

	/**
	 * Debug function to disable all motors that are controlled by this class
	 */
	public void disableShooter(){
		disableRightShooterMotor();
	}

	public void disableRightShooterMotor() { //Was disableShooterMotor2
		rightShooter.set(0.00);
		leftShooter.set(0.00);
	}

	/**
	 * Function to display all the different motor RPM's
	 * Doesn't really apply to the hood motor
	 */
	public void displayRPMValues() {
		System.out.println("Shooter 1 RPM: " + shooter1RPM());
		System.out.println("Shooter 2 RPM: " + shooter2RPM());
	}

	private double shooter1RPM() {
		double rpm;
		rpm = getabsRPM(LEFT_SHOOTER_ID);
		
		return rpm;
	}

	private double shooter2RPM() {
		double rpm;
		rpm = getabsRPM(RIGHT_SHOOTER_ID);
		
		return rpm;
	}

	/**
	 * Gets the abs RPM of the passed motor
	 * @return absRPM
	 */
	private double getabsRPM(int MOTOR_CAN_ID) {
		double rpm;
		double absRPM;

		if (MOTOR_CAN_ID == LEFT_SHOOTER_ID) {
			rpm = leftShooterEncoder.getVelocity();
		}
		else if (MOTOR_CAN_ID == RIGHT_SHOOTER_ID) {
			rpm = rightShooterEncoder.getVelocity();
		}
		else {
			//It should never come to this case
			rpm = 0;
		}

		absRPM = Math.abs(rpm);

		return absRPM;
	}

   /****************************************************************************************** 
   *
   *    Test functions for shooter 
   * 
   ******************************************************************************************/
	public void testShootMotors(double power) {
		//Shooter motor 1 (left motor) needs to be negative to shoot a ball
		//Shooter motor 2 (right motor) needs to be positive to shoot a ball
		//leftShooter.set(-power);
		leftShooter.follow(rightShooter, true); //put in the constructor
		rightShooter.set(power);
		System.out.println("Shooter motor power: " + rightShooter.getOutputCurrent());
	}
} //End of the Shooter Class