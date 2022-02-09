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
	//Object creation
	Drive drive;

	//Network Table: Tracking
	private NetworkTable TrackingValues;

	//Network Table Entries: Tracking
	private NetworkTableEntry isRedAlliance;
	private NetworkTableEntry isEmpty;
	private NetworkTableEntry target;
	private NetworkTableEntry empty;

	//Variables
	private double deadZoneCount = 0.00;
	private double centerX    = 0.00;

	//CONSTANTS
	private static final int IMG_WIDTH = 640;
	//private static final int IMG_HEIGHT = 480;

	//PID controller
	private PIDController cargoController;

	//PID tolerance
	double cargoToleranceDegrees = 2.0f;

	//Cargo Controller
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
		//Variables
		double  turnAngle;
		double  m_CargoCalculatedPower = 0;
		boolean isFull = isEmpty.getBoolean(false);

		//Calls the cargoDetection method
		turnAngle = cargoDetection();

		//Clamps turnAngle
		turnAngle = MathUtil.clamp(turnAngle, -180.00, 180.00);

		if (isFull == true) {
			//Rotate with PID
			m_CargoCalculatedPower = cargoController.calculate(turnAngle, 0.00);
			m_CargoCalculatedPower = MathUtil.clamp(m_CargoCalculatedPower, -0.50, 0.50);

			// Negates power beacuse it's going the wrong way
			m_CargoCalculatedPower = m_CargoCalculatedPower * -1;

			drive.teleopRotate(m_CargoCalculatedPower);
		}
		else if (isFull == false) {
			//Doesn't do anything to prevent constant occilation
		}
		else {
			//Should never even occur
			drive.teleopRotate(0.00);
		}
	}

	/**
	 * Detects the cargo of specified color and provides how far off the robot is
	 * @return turnAngle
	 */
	private double cargoDetection() {
		//Variables
		final int DEAD_ZONE = 25; //Creates a 25 pixel dead zone on either side of the camera's FOV
		boolean pipelineEmpty;
		double  emptyCount;
		double  drivePower;
		double  turn;
	
		//Network Tables
		isEmpty = TrackingValues.getEntry("IsEmpty");
		target = TrackingValues.getEntry("CenterX");
		empty  = TrackingValues.getEntry("Empty");
	
		//Sets the double variables
		pipelineEmpty = isEmpty.getBoolean(false);
		centerX       = target.getDouble(0.00);
		emptyCount    = empty.getDouble(0.00);
	
		//Ignores the 25 pixels on the edges
		if ( (centerX < DEAD_ZONE) || (centerX > IMG_WIDTH - DEAD_ZONE) ) {
		  pipelineEmpty = true;
		  deadZoneCount++;
		}
	
		if (pipelineEmpty == true) {
		  //Prints the emptyCount
		  System.out.println("Empty Count: " + emptyCount + " Dead Zone " + deadZoneCount);

		  //Sets turn to 0
		  turn = 0.00;
		}
		else if (pipelineEmpty == false) {
		  //Does the math for tracking the balls
		  turn = centerX - (IMG_WIDTH / 2);
	
		  //Drive Power
		  drivePower = turn * 0.001;
	
		  System.out.println("Turn: " + turn + " CenterX: " + centerX + " drive: " + drivePower);
	
		  //Resets empty counters
		  emptyCount    = 0;
		  deadZoneCount = 0;
		}
		else {
		  //Sets the values to 0 if otherwise
		  emptyCount = 0.00;
		  turn       = 0.00;
		  centerX    = 0.00;
		  drivePower = 0.00;
		}
    
		return turn;
	}

	/**
	 * Determines if the alliance color is red or not
	 * @param isRed
	 */
	public void setRedAlliance(boolean isRed) {
		//Sets the NetworkTable variable color to the selected alliance color
		isRedAlliance.setBoolean(isRed);
	}
	
	/**
	 * TEST FUNCTIONS
	 */
	/**
	 * Returns the isEmpty entry
	 * @return isEmpty
	 */
	public boolean checkPieplineEmpty() {
		return empty.getBoolean(true);
	}

	/**
	 * Returns the isRedAlliance entry
	 * @return
	 */
	public boolean checkRedAlliance() {
		return isRedAlliance.getBoolean(false);
	}

	/**
	 * Returns the emptyCount entry
	 * @return
	 */
	public double emptyCounter() {
		return empty.getDouble(0.00);
	}

	/**
	 * Returns the centerX entry
	 * @return
	 */
	public double targetPosition() {
		return target.getDouble(0.00);
	}

}

//End of the CargoTracking class