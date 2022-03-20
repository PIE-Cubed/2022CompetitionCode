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

	// CONSTANTS
	private final int LED_PWM_CHANNEL = 0;

	// Object creation
	private Spark ledController;

	// Constructor
	private LedLights()  {
		// Creates an instance for the Spark Controler
		ledController = new Spark(LED_PWM_CHANNEL);

		// Sets the default color to team colors
		teamColors();
	}

	/**
	 * DEFAULT
	 */
	public void defaultMode( boolean isRedAlliance) {
		if (isRedAlliance == true) {
			// Sets our colors to red
			redAlliance();
		}
		else if (isRedAlliance == false) {
			// Sets our colors to blue
			blueAlliance();
		}
		else {
			// Sets our colors to Team Colors
			teamColors();
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

	public void teamColors() {
		// Sparkle blue on gold
		ledController.set(0.53);
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
		ledController.set(-0.25);
	}

	public void onTarget() {
		// Solid color set to Green
		ledController.set(0.77);
	}
		// INCOMPLETE - Input location unknown

	/**
	 * LIMELIGHT ERROR CODES
	 */
	public void limelightFinished() {
		// Solid Green
		ledController.set(0.77);
	}

	public void limelightAdjusting() {
		// Solid Yellow
		ledController.set(0.69);
	}

	public void limelightNoValidTarget() {
		// Solid Red
		ledController.set(0.61);
	}
		// No Valid Target may be in the wrong spot

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