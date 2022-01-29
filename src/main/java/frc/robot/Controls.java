package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Start of class
 */
public class Controls {
    
    //Singleton Method to insure that there is ever only one instance of Controls
    private static Controls instance = null;

    public static synchronized Controls getInstance() {
        if (instance == null) {
            instance = new Controls();
        }

        return instance;
    }

    /**
     * Enumerator for controller ID's
     */
    private enum ControllerIDs {
        JOYSTICK(1),
        XBOX_MANIP_CONTROLLER(0);
        
        private int id;

        // Each item in the enum will now have to be instantiated with a constructor with the integer id. Look few lines above, where JOYSTICK(int id) and XBOXCONTROLLER(int id) are. These are what the constructor is for.
        private ControllerIDs(int id) {
            this.id = id;
        }

        private int getId() {
            return this.id;
        }
    }

    // Controller Object Declaration
    private Joystick joystick;
    private XboxController xboxController;

    private Controls() {
        //Instance Creation
        joystick = new Joystick(ControllerIDs.JOYSTICK.getId());
        xboxController = new XboxController(ControllerIDs.XBOX_MANIP_CONTROLLER.getId());
    }

    /**
     * JOYSTICK DRIVE VALUES
     */
    private double getX() {
        return joystick.getX();
    }

    private double getY() {
        return joystick.getY();
    }

    private double getZ() {
        return joystick.getZ();
    }


    // SHOOTER ENABLED
    private boolean getShooterEnable() {
        return joystick.getTrigger();        
    }

    
    /**
     * 0 degrees is forward on the Joystick
     * this method returns values from -180 to +180
     * @return driveAngle
     */
    public double getDriveAngle() {
        //Gets X and Y positions
        double x = getX();
        double y = getY();
        
        //Does math to figure out the drive angle
        double rad = Math.atan2(x, y);
        double deg = Math.toDegrees(rad);

        return deg;
    }

    /**
     * Positive values are from clockwise rotation 
     * and negative values are from counter-clockwise
     * @return rotatePower
     */
    public double getRotatePower() {
        //double deadZone = 0.3;
        double power = getZ();

        //Halves the power because the rotate is SUPER sensitive
        power = Math.pow(power, 3.0); 
        power = MathUtil.clamp(power, -.5, .5);
            
        return power;    
    }

    /**
     * Gets the drive power
     * @return drivePower
     */
    public double getDrivePower() {
        //Gets X and Y positions
        double x = getX();
        double y = getY() * -1;

        //Does math to calculate the power we want if on an angle 
        double hyp = Math.sqrt(x*x + y*y);
        double hypClamp = MathUtil.clamp(hyp, -1, 1);

        return hypClamp;
    }

    /**
     * Gets the drive X
     * @return driveX
     */
    public double getDriveX() {
        double power = getX();

        return power;
    }

    /**
     * Gets the drive Y
     * @return driveY
     */
    public double getDriveY() {
        double power = getY() * -1;

        return power;
    }


    /**
     * These are all Functions of the Xbox controller
     */
    /**
     * Start Button Pressed
     * WHETER TO KILL ALL ACTIVE AUTO PROGRAMS!
     * @return startButtonPressed
     */
    public boolean autoKill() {
        return xboxController.getStartButtonPressed();
    }
    
    /**
     * Button A
     * @return buttonAPressed
     */
    public boolean grabberDeployRetract() {
        return xboxController.getAButtonPressed();
    }
    //

    /**
     * Button B
     * @return buttonBPressed
     */
    //

    /**
     * Button X
     * @return buttonXPressed
     */
    //

    /**
     * Button Y
     * @return buttonYPressed
     */
    public boolean getXboxY() {
        return xboxController.getYButtonPressed();
    }

    

    /**
     * Right Bumper Pressed
     * @return rightBumperPresed
     */
    public boolean getRightBumper() {
        return xboxController.getRightBumper();
    }

    /**
     * Left Bumper Pressed
     * @return leftBumperPresed
     */
    public boolean getLeftBumper() {
        return xboxController.getLeftBumper();
    }

    /**
     * Right trigger
     */
    public double getRightTrigger() {
        double power;
        power = xboxController.getRightTriggerAxis();

        //trigger dead band
        if (power > 0.1) {
            return power;
        }
        else {
            return 0;
        }
    }

    //Left trigger
    public double getLeftTrigger() {
        double power = xboxController.getLeftTriggerAxis();

        if (power > 0.1) {
            return power;
        }
        else {
            return 0;
        }
    }
}

// End of the Controls class