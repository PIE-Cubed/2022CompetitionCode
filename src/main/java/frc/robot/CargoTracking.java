package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.math.controller.PIDController;

/**
 * Start of class
 */
public class CargoTracking {
	//Object creation
	Drive drive;
  Controls controls;

	//Network Tables
	NetworkTable TrackingValues;

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
	public CargoTracking() {
		// Instance creation
    controls = Controls.getInstance();
		//drive = Drive.getInstance();

		// Creates Network Tables instance
		TrackingValues = NetworkTableInstance.getDefault().getTable("TrackingValues");

		// Creates a PID controller
		cargoController = new PIDController(cP, cI, cD);
    cargoController.setTolerance(cargoToleranceDegrees);
    cargoController.enableContinuousInput(-180.0, 180.0);
	}

	/**
	 * Method to turn and face the cargo of selected color
	 */
	public void faceCargo() {
		//Variables
		double m_CargoCalculatedPower = 0;
		double turnAngle;

		//Calls the cargoDetection method
		turnAngle = cargoDetection();

		//Clamps turnAngle
		turnAngle = MathUtil.clamp(turnAngle, -180.00, 180.00);

		//Rotate with PID
		m_CargoCalculatedPower = cargoController.calculate(turnAngle, 0.00);
    m_CargoCalculatedPower = MathUtil.clamp(m_CargoCalculatedPower, -0.50, 0.50);
		drive.teleopRotate(m_CargoCalculatedPower);
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
		NetworkTableEntry isEmpty = TrackingValues.getEntry("IsEmpty");
		NetworkTableEntry target = TrackingValues.getEntry("CenterX");
		NetworkTableEntry empty  = TrackingValues.getEntry("Empty");
	
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
   * Sets the alliance and target cargo color
   */
  public void setCargoColor(String color) {
    //Sets the NetworkTable variable color to alliance color
    NetworkTableEntry targetColor = TrackingValues.getEntry("TargetColor");
    targetColor.setString(color);
  }

}

//End of the CargoTracking class