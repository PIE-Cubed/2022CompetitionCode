package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
	//2 holes above marked one
	//Low shot against hub- 0.25 speed, 1380-1500 rpm
	//High shot from edge of tarmac- 0.5 speed, 3800-4000 rpm

	//3 holes above marked one
	//Low against hub- 0.3 speed, 1650-1750 rpm
	//High from edge of tarmac- 0.525 speed, 2980-3100 rpm

	
	// SPARK MAX
	private CANSparkMax frontShooter;
	private CANSparkMax rearShooter;

	// SPARK MAX ID's
	private int FRONT_SHOOTER_ID  = 19; //front
	private int REAR_SHOOTER_ID = 20; //right-->rear

	//Double solenoid
	private DoubleSolenoid feeder;
	private int FEEDER_DEPLOY_ID  = 3;
	private int FEEDER_RETRACT_ID = 7;

	// Encoders
	private RelativeEncoder frontShooterEncoder;
	private RelativeEncoder rearShooterEncoder;

	// POWER CONSTANTS
	public final double OFF_POWER  = 0.00;

	public final double HIGH_SHOT_REAR_POWER   = 0.525;
	public final double HIGH_SHOT_FRONT_POWER  = -0.525;

	public final double LOW_SHOT_REAR_POWER    = 0.25;
	public final double LOW_SHOT_FRONT_POWER   = -0.25;

	public final double LAUNCH_PAD_REAR_POWER  = 0.65;
	public final double LAUNCH_PAD_FRONT_POWER = -0.65;

	public final double AUTO_RING_REAR_POWER   = 0.6;
	public final double AUTO_RING_FRONT_POWER  = -0.6;

	// RPM CONSTANTS
	public final double OFF_TARGET_RPM              = 0;

	public final double HIGH_SHOT_REAR_TARGET_RPM   = 2980;
	public final double HIGH_SHOT_FRONT_TARGET_RPM  = 2980;

	public final double LOW_SHOT_REAR_TARGET_RPM    = 1650;
	public final double LOW_SHOT_FRONT_TARGET_RPM   = 1650;

	public final double LAUNCH_PAD_REAR_TARGET_RPM  = 3500;
	public final double LAUNCH_PAD_FRONT_TARGET_RPM = 3500;

	public final double AUTO_RING_REAR_TARGET_RPM   = 3300;
	public final double AUTO_RING_FRONT_TARGET_RPM  = 3300;

	// Current Limit Constants
	private static final int SHOOTER_CURRENT_LIMIT = 80;

	// Variables
	public  double                frontTargetVelocity;
	public  double                rearTargetVelocity;
	private int                   targetCount        = 0;
	private int                   noTargetCount      = 0;
	private double                frontPower         = 0;
	private double                rearPower          = 0;
	private boolean               feederDeployed     = false;

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
		frontShooter  = new CANSparkMax(FRONT_SHOOTER_ID, MotorType.kBrushless); //Shooter 1 requires negative power to shoot
		rearShooter   = new CANSparkMax(REAR_SHOOTER_ID, MotorType.kBrushless); //Shooter 2 requires positive power to shoot

		//Create double solenoid and retract it

		// Sets the current limtis for the motors
		frontShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
		rearShooter .setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

		// Sets the mode of the motors (if this works in the code)
		frontShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rearShooter .setIdleMode(CANSparkMax.IdleMode.kCoast);
		
		// Set Shooter related motors to off to Start the Match
		frontShooter.set(0.0);
		rearShooter .set(0.0);

		// Encoders
		frontShooterEncoder = frontShooter.getEncoder();
		rearShooterEncoder  = rearShooter.getEncoder();

		// PID Controller
		shooterController = new PIDController(kP, kI, kD);
	}


	/****************************************************************************************** 
    *
    *    autoShooterControl()
	*    Uses PID to get shooter to appropriate speed for given shot
    *   
	******************************************************************************************/
	public void autoShooterControl(ShootLocation location) {
		double frontPowerError;
		double rearPowerError;
		
		if (location == ShootLocation.HIGH_SHOT) {
			frontPowerError     = shooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), HIGH_SHOT_FRONT_TARGET_RPM);
			rearPowerError      = shooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , HIGH_SHOT_REAR_TARGET_RPM);
			frontTargetVelocity = HIGH_SHOT_FRONT_TARGET_RPM;
			rearTargetVelocity  = HIGH_SHOT_REAR_TARGET_RPM;
			frontPower          = HIGH_SHOT_FRONT_POWER;
			rearPower           = HIGH_SHOT_REAR_POWER;
		}
		else if (location == ShootLocation.LOW_SHOT) {
			frontPowerError     = shooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), LOW_SHOT_FRONT_TARGET_RPM);
			rearPowerError      = shooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , LOW_SHOT_REAR_TARGET_RPM);
			frontTargetVelocity = LOW_SHOT_FRONT_TARGET_RPM;
			rearTargetVelocity  = LOW_SHOT_REAR_TARGET_RPM;
			frontPower          = LOW_SHOT_FRONT_POWER;
			rearPower           = LOW_SHOT_REAR_POWER;
		}
		else if (location == ShootLocation.LAUNCH_PAD) {
			frontPowerError     = shooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), LAUNCH_PAD_FRONT_TARGET_RPM);
			rearPowerError      = shooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , LAUNCH_PAD_REAR_TARGET_RPM);
			frontTargetVelocity = LAUNCH_PAD_FRONT_TARGET_RPM;
			rearTargetVelocity  = LAUNCH_PAD_REAR_TARGET_RPM;
			frontPower          = LAUNCH_PAD_FRONT_POWER;
			rearPower           = LAUNCH_PAD_REAR_POWER;
		}
		else if (location == ShootLocation.AUTO_RING) {
			frontPowerError     = shooterController.calculate( getabsRPM(FRONT_SHOOTER_ID), AUTO_RING_FRONT_TARGET_RPM);
			rearPowerError      = shooterController.calculate( getabsRPM(REAR_SHOOTER_ID) , AUTO_RING_REAR_TARGET_RPM);
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

		//Increments shooter powers by PID-calculated error
		frontPower = frontPower - frontPowerError;
		frontPower = MathUtil.clamp(frontPower, -1.0, 0.0);
		rearPower  = rearPower + rearPowerError;
		rearPower  = MathUtil.clamp(rearPower, 0.0, 1.0);

		//Displays powers and rpms to smartdashboard
		SmartDashboard.putNumber("Front power", frontPower);
		SmartDashboard.putNumber("Front rpm", getabsRPM(FRONT_SHOOTER_ID));
		SmartDashboard.putNumber("Rear power", rearPower);
		SmartDashboard.putNumber("Rear rpm", getabsRPM(REAR_SHOOTER_ID));

		frontShooter.set(frontPower);
		rearShooter.set(rearPower);
	}


	/****************************************************************************************** 
    *
    *    manualShooterControl()
	*    Gets shooter to appropriate speed without PID
    *   
    ******************************************************************************************/
	public void manualShooterControl(ShootLocation location) {
											
	if (location == ShootLocation.OFF) {
			rearShooter .set(OFF_POWER);
			frontShooter.set(OFF_POWER);
			rearTargetVelocity  = OFF_TARGET_RPM;
			frontTargetVelocity = OFF_TARGET_RPM;
		}
		else if (location == ShootLocation.HIGH_SHOT) {
			rearShooter .set(HIGH_SHOT_REAR_POWER);
			frontShooter.set(HIGH_SHOT_FRONT_POWER);
			rearTargetVelocity  = HIGH_SHOT_REAR_TARGET_RPM;
			frontTargetVelocity = HIGH_SHOT_FRONT_TARGET_RPM;
		}
		else if (location == ShootLocation.LOW_SHOT) {
			rearShooter .set(LOW_SHOT_REAR_POWER);
			frontShooter.set(LOW_SHOT_FRONT_POWER);
			rearTargetVelocity  = LOW_SHOT_REAR_TARGET_RPM;
			frontTargetVelocity = LOW_SHOT_FRONT_TARGET_RPM;
		}
		else if (location == ShootLocation.LAUNCH_PAD) {
			rearShooter .set(LAUNCH_PAD_REAR_POWER);
			frontShooter.set(LAUNCH_PAD_FRONT_POWER);
			rearTargetVelocity  = LAUNCH_PAD_REAR_TARGET_RPM;
			frontTargetVelocity = LAUNCH_PAD_FRONT_TARGET_RPM;
		}
		else if (location == ShootLocation.AUTO_RING) {
			rearShooter .set(AUTO_RING_REAR_POWER);
			frontShooter.set(AUTO_RING_FRONT_POWER);
			rearTargetVelocity  = AUTO_RING_REAR_TARGET_RPM;
			frontTargetVelocity = AUTO_RING_FRONT_TARGET_RPM;
		}
		else {
			rearShooter.set(OFF_POWER);
			frontShooter.set(OFF_POWER);
			rearTargetVelocity  = OFF_TARGET_RPM;
			frontTargetVelocity = OFF_TARGET_RPM;
		}
	}


	/****************************************************************************************** 
    *
    *    shooterReady()
	*    Checks if shooter is ready to fire
    *   
    ******************************************************************************************/
	public boolean shooterReady() {
		double rearRpm;
		double frontRpm;
		rearRpm  = getabsRPM(REAR_SHOOTER_ID);
		frontRpm = getabsRPM(FRONT_SHOOTER_ID);

		double rearLowerLimit  = rearTargetVelocity  - 100;
		double rearUpperLimit  = rearTargetVelocity  + 100;
		double frontLowerLimit = frontTargetVelocity - 100;
		double frontUpperLimit = frontTargetVelocity + 100;

		if ((rearRpm  > rearLowerLimit  && rearRpm < rearUpperLimit) && 
			(frontRpm > frontLowerLimit && frontRpm < frontUpperLimit))  {
			targetCount ++;
			noTargetCount = 0;
			
			if(targetCount >= 5) { 
				return true;
			}
			else {
				return false;
			}
		}
		else {
			noTargetCount ++;
			targetCount = 0;

			//Timeout, shoots to clear system, better than holding ball
			if (noTargetCount >= 300) {
				System.out.println("Not up to speed. Check battery or PID");
				System.out.println("Front rpm: " + frontRpm + "front target rpm: " + frontTargetVelocity);
				System.out.println("Rear rpm: " + rearRpm + "rear target rpm: " + rearTargetVelocity);

				//Resets noTargetCount once it has been shooting for 150 iterations
				if (noTargetCount >= 450) {
					noTargetCount = 0;
					return false;
				}
				return true;
			}
			else {
				return false;
			}
		}
	}

	/****************************************************************************************** 
    *
    *    deployFeeder()
	*    Deploys feeder
    *   
    ******************************************************************************************/
	public void deployFeeder() {
		//deploy feeder if not already deployed
	}

	/****************************************************************************************** 
    *
    *    retractFeeder()
	*    Retracts feeder
    *   
    ******************************************************************************************/
	public void retractFeeder() {
		//retract feeder if not already retracted
	}

	/**
	 * DEBUG / TEST FUNCTIONS
	 */
	
	 /**
	 * A debug function for the shooter
	 * @param power
	 */
	public void testShoooter(double power) {
		frontShooter.set(power);
		rearShooter .set(power);
		System.out.println("Power: " + power + " RPM: " + getabsRPM(FRONT_SHOOTER_ID));
	}

	public void enableShooterFullPower() {
		frontShooter.set(1.00);
		rearShooter.set(1.00);

		System.out.println("RPM 1: " + getabsRPM(FRONT_SHOOTER_ID));
		System.out.println("RPM 2: " + getabsRPM(REAR_SHOOTER_ID));
	}

	/**
	 * Debug function to disable all motors that are controlled by this class
	 */
	public void disableShooter(){
		frontShooter.set(0);
		rearShooter.set(0);
		retractFeeder();
	}

	public void disableRearShooterMotor() { //Was disableShooterMotor2
		rearShooter .set(0.00);
		frontShooter.set(0.00);
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
	private double getabsRPM(int MOTOR_CAN_ID) {
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

   /****************************************************************************************** 
   *
   *    Test functions for shooter 
   * 
   ******************************************************************************************/
	public void testShootMotors(double powerF, double powerR) {
		//Shooter motor 1 (front motor) needs to be negative to shoot a ball
		//Shooter motor 2 (rear motor) needs to be positive to shoot a ball
		frontShooter.set(-powerF);
		rearShooter.set(powerR);
		System.out.println("Front RPM: " + getabsRPM(FRONT_SHOOTER_ID) + " Rear RPM: " + getabsRPM(REAR_SHOOTER_ID));
	}
} //End of the Shooter Class