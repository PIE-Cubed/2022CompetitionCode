package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.networktables.*;

import edu.wpi.first.math.controller.PIDController;

/**
 * Start of class
 */
public class CargoTracking {
	// Object creation
	Drive drive;

	// Network Table
	private NetworkTable TrackingValues;

	// Network Table Entries
	private NetworkTableEntry isRedAlliance;
	private NetworkTableEntry isEmpty;
	private NetworkTableEntry target;
	private NetworkTableEntry empty;

	// Variables
	private double deadZoneCount = 0.00;
	private double centerX    = 0.00;

	// CONSTANTS
	private static final int IMG_WIDTH_RAW    = 160;
	private static final int IMG_SCALE_FACTOR = 3; //Can be derived from the Raspberry Pi Code
	private static final int IMG_WIDTH_SCALED = IMG_WIDTH_RAW * IMG_SCALE_FACTOR;
	private static final int DIST_IMG_LEFT    = (IMG_WIDTH_SCALED / 2) * -1; //Distance to the right of the screen
	private static final int DIST_IMG_RIGHT   = (IMG_WIDTH_SCALED / 2);      //Distance to the left of the screen
	//private static final int IMG_HEIGHT = 480;

	// PID controller
	private PIDController cargoController;

	// PID tolerance
	double cargoToleranceDegrees = 1.5f;

	// Cargo Controller
	private static final double cP = 0.02;
	private static final double cI = 0.01;
	private static final double cD = 0.01;

	/**
	 * CONSTRUCTOR
	 */
	public CargoTracking(Drive drive) {
		// Instance creation
		this.drive = drive;

		// Creates a PID controller
		cargoController = new PIDController(cP, cI, cD);
		cargoController.setTolerance(cargoToleranceDegrees);
		cargoController.enableContinuousInput(-180.0, 180.0);

		// Creates a Network Tables instance
		TrackingValues = NetworkTableInstance.getDefault().getTable("TrackingValues");

		// Creates the Networktable Entries
		isRedAlliance = TrackingValues.getEntry("isRedAlliance"); // Boolean
		isEmpty       = TrackingValues.getEntry("IsEmpty");       // Boolean
		target        = TrackingValues.getEntry("CenterX");       // Double
		empty         = TrackingValues.getEntry("Empty");         // Double
	}

	/**
	 * Method to turn and face the cargo of selected color
	 */
	public void faceCargo() {
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
	
		// Ignores the 25 pixels on the edges
		if ( (centerX < (DIST_IMG_LEFT + DEAD_ZONE)) || (centerX > (DIST_IMG_RIGHT - DEAD_ZONE)) )  {
		  pipelineEmpty = true;
		  deadZoneCount++;
		}
	
		if (pipelineEmpty == true) {
		  // Prints the emptyCount
		  System.out.println("Empty Count: " + emptyCount + " Dead Zone " + deadZoneCount);

		  // Sets turn to 0.00
		  turn = 0.00;
		}
		else if (pipelineEmpty == false) {
		  // Does the math for tracking the balls
		  turn = centerX - (IMG_WIDTH_SCALED / 2);

		  // Prints the values
		  //System.out.println("CenterX: " + centerX + " TurnPower: " + turn);
	
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
	 * TEST FUNCTIONS
	 */

}

//End of the CargoTracking class