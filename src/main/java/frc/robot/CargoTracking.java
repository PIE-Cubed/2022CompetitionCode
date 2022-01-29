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
	//Network Tables
	NetworkTable TrackingValues;
	NetworkTableEntry targetColor;

	//Variables
	private double deadZoneCount = 0.00;
	private double centerX    = 0.00;

	//CONSTANTS
	private static final int IMG_WIDTH = 640;
	//private static final int IMG_HEIGHT = 480;

	/**
	 * CONSTRUCTOR
	 */
	public CargoTracking() {
		// Creates Network Tables instance
		TrackingValues = NetworkTableInstance.getDefault().getTable("TrackingValues");

		//
		NetworkTableEntry targetColor = TrackingValues.getEntry("TargetColor");
    	targetColor.setDefaultString("Default");
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
    //Sets the NetworkTable variable color to the selected alliance color
    //NetworkTableEntry targetColor = TrackingValues.getEntry("TargetColor");
    targetColor.setString(color);
  }

}

//End of the CargoTracking class