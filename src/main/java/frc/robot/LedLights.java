package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * Start of class
 */
public class LedLights {
	// Singleton for LedLights because it is used in many places
	public static LedLights instance = null;
	public static synchronized LedLights getInstance() {
		if (instance == null) {
			instance = new LedLights();
		}

		return instance;
	}

	// Variables
	private boolean shooterOnTarget;
	private boolean limelightOnTarget;
	private boolean limelightNoTarget;

	// CONSTANTS
	private final int LED_PWM_CHANNEL = 0;

	// Object creation
	private Spark ledController;

	// Constructor
	private LedLights()  {
		// Creates an instance for the Spark Controler
		ledController = new Spark(LED_PWM_CHANNEL);

		// Sets variables
		shooterOnTarget   = false;
		limelightOnTarget = false;
		limelightNoTarget = true;

		// Sets the default color to team colors
		teamColors();
	}

	/**
	 * Team Colors
	 */
	public void teamColors() {
		// Sparkle blue on gold
		//ledController.set(.53).
		ledController.set(0.41);
	}

	/**
	 * DEFAULT
	 */
	public void defaultMode( boolean isRedAlliance) {
		if (isRedAlliance == true) {
			// Sets our colors to red
			//redAlliance();
		}
		else if (isRedAlliance == false) {
			// Sets our colors to blue
			//blueAlliance();
		}
	}

	public void redAlliance(){
		// Heartbeat Red
		ledController.set(-0.25);
	}

	public void blueAlliance(){
		// Heartbeat Blue
		ledController.set(-0.23);
	}

	/**
	 * AUTO MODES
	 */
	public void autoMode() {
		// Solid color set to Aqua
		ledController.set(0.81);
	}

	public void autoModeFinished() {
		// Solid color set to Gold
		ledController.set(0.67);
	}

	/**
	 * SHOOTER
	 */
	public void shooterReady() {
		// Heart Beat Red
		//ledController.set(-0.25);
		shooterOnTarget = true;
	}

	public void shooterNotReady() {
		shooterOnTarget = false;
	}


	/**
	 * LIMELIGHT ERROR CODES
	 */
	public void limelightFinished() {
		// Solid Green
		//ledController.set(0.77);
		limelightOnTarget = true;
	}

	public void limelightAdjusting() {
		// Solid Yellow
		ledController.set(0.69);
		limelightOnTarget = false;
	}

	public void limelightNoValidTarget() {
		// Solid Red
		//ledController.set(0.61);
		limelightNoTarget = true;
		limelightOnTarget = false;
	}
		// No Valid Target may be in the wrong spot
	
	public void updateShooter() {
		if (shooterOnTarget && limelightOnTarget) {
			//Pure green
			ledController.set(0.77);
		}
		else if (shooterOnTarget && limelightNoTarget) {
			//Green and red
			ledController.set(-0.85);
		}
		else if (shooterOnTarget) {
			//Green and yellow
			ledController.set(-0.91);
		}
		else if (!shooterOnTarget && limelightNoTarget) {
			//Red
			ledController.set(-0.11);//-0.85
		}
		else if (!shooterOnTarget && limelightOnTarget) {
			//Red and green, more red though
		}
		else if (!shooterOnTarget) {
			//Red and yellow
			ledController.set(0.65);//-0.93?
		}
	}

	/**
	 * OBJECT TRACKING ERROR CODES
	 */
	public void cargoTrackingFinished() {
		// Solid Green
		ledController.set(0.77);
	}

	public void cargoTrackingAdjusting() {
		// Solid Yellow
		ledController.set(0.69);
	}

	public void cargoTrackingNoTarget() {
		// Solid Red
		ledController.set(0.61);
	}

}

// End of the Ledligths Class