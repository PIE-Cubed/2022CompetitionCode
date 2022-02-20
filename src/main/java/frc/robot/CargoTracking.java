package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.networktables.*;

/**
 * A class that tracks cargo with the help of a Raspberry Pi
 */
public class CargoTracking {
	// firstTime
	private boolean cargoFirstTime = true;

	// Object creation
	private Drive drive;

	// Network Table
	private NetworkTable TrackingValues;

	// Network Table Entries
	private NetworkTableEntry isRedAlliance;
	private NetworkTableEntry isEmpty;
	private NetworkTableEntry target;
	private NetworkTableEntry empty;

	// Variables
	private double deadZoneCount  = 0.00;
	private double centerX        = 0.00;

	// Auto Tracking Variables
	private int noTargetCount     = 0;
	private int targetLockedCount = 0;
	private long timeOut = 0;

	// CONSTANTS
	private static final int IMG_WIDTH_RAW    = 160;
	private static final int IMG_SCALE_FACTOR = 3;   //Can be derived from the Raspberry Pi Code
	private static final int IMG_WIDTH_SCALED = IMG_WIDTH_RAW * IMG_SCALE_FACTOR;
	//private static final int IMG_HEIGHT = 480;

	// Auto Tracking Constants
	private final int  TIME_OUT_SEC  = 5;
	private final long TIME_OUT_MSEC = TIME_OUT_SEC * 1000;
	private final int  FAIL_COUNT      = 10;
	private final int  ON_TARGET_COUNT = 5;

	// PID controller
	private static PIDController cargoController;

	// PID tolerance
	private double cargoToleranceDegrees = 1.0f;

	// Cargo Controller
	private static final double cP = 0.003; //0.0025
	private static final double cI = 0.0001;
	private static final double cD = 0.000;

	// Integrator limits
	private static final double MIN_INTEGRATOR = -0.050; //-0.075
	private static final double MAX_INTEGRATOR =  0.050; // 0.075

	/**
	 * CONSTRUCTOR
	 * @param drive
	 */
	public CargoTracking(Drive drive) {
		// Instance creation
		this.drive = drive;

		// Creates a PID controller
		cargoController = new PIDController(cP, cI, cD);
		cargoController.setTolerance(cargoToleranceDegrees);
		cargoController.enableContinuousInput(-180.0, 180.0);

		//Restricts the PID's integrator range
		cargoController.setIntegratorRange(MIN_INTEGRATOR, MAX_INTEGRATOR);

		// Creates a Network Tables instance
		TrackingValues = NetworkTableInstance.getDefault().getTable("TrackingValues");

		// Creates the Networktable Entries
		isRedAlliance = TrackingValues.getEntry("isRedAlliance"); // Boolean
		isEmpty       = TrackingValues.getEntry("IsEmpty");       // Boolean
		target        = TrackingValues.getEntry("CenterX");       // Double
		empty         = TrackingValues.getEntry("Empty");         // Double
	}

	/**
	 * 
	 * @return Targetting Status
	 */
	public int autoCargoTrack() {
		// Variables
		long    currentMs = System.currentTimeMillis();
		boolean noTarget  = isEmpty.getBoolean(true);

		// Runs the firstTime procedure
		if (cargoFirstTime == true) {
			//Sets firstTime to false
			cargoFirstTime = false;

			//Sets the timeOut
			timeOut = currentMs + TIME_OUT_MSEC;
			System.out.println("CargoTracking TimeOut: " + timeOut);
		}

		if (noTarget == true) {
			//Increments the noTargetCount
			noTargetCount++;

			if (noTargetCount <= FAIL_COUNT) {
				// Robot continues searching
				faceCargo();

				// Returns the error code for continue
				return Robot.CONT;
			}
			else {
				// Resets variables
				noTargetCount = 0;
				targetLockedCount = 0;
				cargoFirstTime = true;
				cargoController.reset();

				// Stops the robot
				drive.stopWheels();

				// Prints telemetry
				System.out.println("Cargo Not Found!");

				// Returns error code for failure
				return Robot.FAIL;
			}
		}

		// Rotates the robot
		faceCargo();

		// If the robot is aat the setpoint, targetLockedCount increses
		if (cargoController.atSetpoint() == true) {
			targetLockedCount++;
		}

		// Checks if the targetLockedCount is greater than our threshold for success
		if (targetLockedCount >= ON_TARGET_COUNT) {
			// Resets variables
			noTargetCount = 0;
			targetLockedCount = 0;
			cargoFirstTime = true;
			cargoController.reset();

			// Stops the robot
			drive.stopWheels();

			// Prints telemetry
			System.out.println("Cargo Found!" + " CenterX: " + centerX);

			// Returns error code for success
			return Robot.DONE;
		}

		// Checks if the robot has passed the timeOut
		if (currentMs > timeOut) {
			// Resets variables
			noTargetCount = 0;
			targetLockedCount = 0;
			cargoFirstTime = true;
			cargoController.reset();

			// Stops the robot
			drive.stopWheels();

			// Prints telemetry
			System.out.println("TimeOut!" + " Cargo Present: " + noTarget);

			// Returns error code for failure
			return Robot.FAIL;
		}

		// Returns error code for continue
		return Robot.CONT;
	}

	/**
	 * Method to turn and face the cargo of selected color
	 */
	private void faceCargo() {
		// Variables
		double  turnAngle;
		double  m_CargoCalculatedPower = 0.00;
		boolean isNotFull = isEmpty.getBoolean(true);

		// Calls the cargoDetection method
		turnAngle = cargoDetection();

		// Clamps turnAngle
		turnAngle = MathUtil.clamp(turnAngle, -180.00, 180.00);

		if (isNotFull == true) {
			// Doesn't do anything to prevent constant occilation
		}
		else if (isNotFull == false) {
			/**
			 * Rotate with PID
			 */
			// Calculate the rotate power
			m_CargoCalculatedPower = cargoController.calculate(turnAngle, 0.00);
			// Scales the power down to account for the image being resized
			m_CargoCalculatedPower = m_CargoCalculatedPower / IMG_SCALE_FACTOR;
			// Clamps the power so the robot doesn't go flying
			m_CargoCalculatedPower = MathUtil.clamp(m_CargoCalculatedPower, -0.50, 0.50);
			// Negates the powwer to make it go in the right direction
			m_CargoCalculatedPower = -1 * m_CargoCalculatedPower;

			// Prints calculated power
			System.out.println("Rotate Power: " + m_CargoCalculatedPower);

			// Starts rotating
			drive.teleopRotate(m_CargoCalculatedPower);
		}
		else {
			// Should never even occur
			drive.teleopRotate(0.00);
		}
	}

	/**
	 * Detects the cargo of specified color and provides how far off the robot is
	 * @return turnAngle
	 */
	private double cargoDetection() {
		//Variables
		final int DEAD_ZONE = 20; // Creates a 20 pixel dead zone on either side of the camera's FOV
		boolean pipelineEmpty;
		double  emptyCount;
		double  turn;

		// Sets the double variables
		pipelineEmpty = isEmpty.getBoolean(true);
		centerX       = target.getDouble(0.00);
		emptyCount    = empty.getDouble(0.00);

		// Ignores the 20 pixels on the edges
		// CenterX is a misleading name beacuse it actually measures from left to right, not from the center
		if ( (centerX < (0 + DEAD_ZONE)) || (centerX > (IMG_WIDTH_SCALED - DEAD_ZONE)) )  {
		  pipelineEmpty = true;
		  deadZoneCount++;
		}

		if (pipelineEmpty == true) {
		  // Prints the emptyCount
		  System.out.println("IsEmpty: " + pipelineEmpty + " Empty Count: " + emptyCount + " Dead Zone " + deadZoneCount);

		  // Sets turn to 0.00
		  turn = 0.00;
		}
		else if (pipelineEmpty == false) {
		  // Does the math for tracking the balls
		  turn = centerX - (IMG_WIDTH_SCALED / 2);

		  // Prints the values
		  System.out.println("IsEmpty: " + pipelineEmpty + " CenterX: " + centerX + " Turn: " + turn);
	
		  // Resets empty counters
		  emptyCount    = 0;
		  deadZoneCount = 0;
		}
		else {
		  // Sets the values to 0 if otherwise
		  emptyCount = 0.00;
		  turn       = 0.00;
		  centerX    = 0.00;
		}
		
		return turn;
	}

	/**
	 * Determines if the alliance color is red or not
	 * @param isRed
	 */
	public void setRedAlliance(boolean isRed) {
		// Sets the NetworkTable variable color to the selected alliance color
		isRedAlliance.setBoolean(isRed);
	}
	
	/**
	 * RETURN FUNCTIONS
	 */
	/**
	 * Checks if there is a valid target
	 * @return validTarget
	 */
	public boolean isTargetValid() {
		boolean validTarget = !isEmpty.getBoolean(true);
		return validTarget;
	}

	/**
	 * Returns distance of center
	 * @return
	 */
	public double getCenterOffset() {
		double error = centerX - (IMG_WIDTH_SCALED / 2);
		return error;
	}
}

//End of the CargoTracking class