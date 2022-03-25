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
	private boolean errorCodeDisplayed;
	private int     delayCount;

	// CONSTANTS
	private final int LED_PWM_CHANNEL = 0;
	private final int LED_DELAY       = 50; 

	// Object creation
	private Spark ledController;

	// Constructor
	private LedLights()  {
		// Creates an instance for the Spark Controler
		ledController = new Spark(LED_PWM_CHANNEL);

		// Sets variables
		shooterOnTarget    = false;
		limelightOnTarget  = false;
		limelightNoTarget  = true;
		errorCodeDisplayed = false;

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
		errorCodeDisplayed = false;
	}

	/**
	 * DEFAULT
	 */
	public void defaultMode( boolean isRedAlliance) {
		// Checkes if an error code has recently been displayed
		if (errorCodeDisplayed == true) {
			// Resets the delay counter and determines if an error code has been displayed
			errorCodeDisplayed = false;
			delayCount = 0;
		}
		else {
			// Increments the counter
			delayCount ++;
		}

		// If passed the delay amount (20 cycles of the code), reverts to alliance colors
		if (delayCount >= LED_DELAY) {
			if (isRedAlliance == true) {
				// Sets our colors to red
				redAlliance();
			}
			else if (isRedAlliance == false) {
				// Sets our colors to blue
				blueAlliance();
			}

			// Resets the counter
			delayCount = 0;
		}
	}

	public void redAlliance(){
		// Heartbeat Red (-0.25)
		ledController.set(-0.25);
		errorCodeDisplayed = false;
	}

	public void blueAlliance(){
		// Heartbeat Blue (-0.23)
		ledController.set(-0.23);
		errorCodeDisplayed = false;
	}

	/**
	 * AUTO MODES
	 */
	public void autoMode() {
		// Solid color set to Aqua
		ledController.set(0.81);
		errorCodeDisplayed = true;
	}

	public void autoModeFinished() {
		// Solid color set to Gold
		ledController.set(0.67);
		errorCodeDisplayed = true;
	}

	/**
	 * SHOOTER
	 */
	public void shooterReady() {
		// Heart Beat Red
		//ledController.set(-0.25);
		shooterOnTarget = true;
		errorCodeDisplayed = true;
	}

	public void shooterNotReady() {
		shooterOnTarget = false;
		errorCodeDisplayed = true;
	}


	/**
	 * LIMELIGHT ERROR CODES
	 */
	public void limelightFinished() {
		// Solid Green
		//ledController.set(0.77);
		limelightOnTarget = true;
		errorCodeDisplayed = true;
	}

	public void limelightAdjusting() {
		// Solid Yellow
		ledController.set(0.69);
		limelightOnTarget = false;
		errorCodeDisplayed = true;
	}

	public void limelightNoValidTarget() {
		// Solid Red
		//ledController.set(0.61);
		limelightNoTarget = true;
		limelightOnTarget = false;
		errorCodeDisplayed = true;
	}
	
	//Green        - shooter up to speed and limelight targeted
	//Yellow       - shooter up to speed, but limelight not targeted
	//Red          - shooter not up to speed
	//Red flashing - limelight and shooter are both off	
	public void updateShooter() {
		if (shooterOnTarget && limelightOnTarget) {
			//Pure green
			ledController.set(0.77);
		}
		else if (shooterOnTarget && limelightNoTarget) {
			//Solid yellow
			ledController.set(0.69);//-0.85
		}
		else if (shooterOnTarget) {
			//Solid yellow
			ledController.set(0.69);//0.73
		}
		else if (!shooterOnTarget && limelightNoTarget) {
			//Red flashing
			ledController.set(-0.11);
		}
		else if (!shooterOnTarget && limelightOnTarget) {
			//Solid red
			ledController.set(0.61);
		}
		else if (!shooterOnTarget) {
			//Red flashing
			ledController.set(-0.11);
		}
	}

	/**
	 * OBJECT TRACKING ERROR CODES
	 */
	public void cargoTrackingFinished() {
		// Solid Green
		ledController.set(0.77);
		errorCodeDisplayed = true;
	}

	public void cargoTrackingAdjusting() {
		// Solid Yellow
		ledController.set(0.69);
		errorCodeDisplayed = true;
	}

	public void cargoTrackingNoTarget() {
		// Solid Red
		ledController.set(0.61);
		errorCodeDisplayed = true;
	}

	/**
	 * CLIMBER ERROR CODES
	 */
	public void climberAtPosition() {
		// Forest twinkle
		ledController.set(-0.47);
		errorCodeDisplayed = true;
	}

	public void climberMoving() {
		// Strobe gold
		ledController.set(-0.07);
		errorCodeDisplayed = true;
	}

	public void climberDone() {
		// Rainbow party palette
		ledController.set(-0.97);
		errorCodeDisplayed = true;
	}

}

// End of the Ledligths Class
