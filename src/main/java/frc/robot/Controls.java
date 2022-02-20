package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Grabber.GrabberDirection;
import frc.robot.Shooter.ShootLocation;

/**
 * Start of class
 */
public class Controls {
    
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

    //Constructor
    public Controls() {
        //Instance Creation
        joystick       = new Joystick(ControllerIDs.JOYSTICK.getId());
        xboxController = new XboxController(ControllerIDs.XBOX_MANIP_CONTROLLER.getId());
    }

    /**
     * JOYSTICK FUNCTIONS
     */
    // SHOOTER ENABLED
    public boolean getShooterEnable() {        
        return joystick.getTrigger();        
    }
    
    /**
     * DRIVE FUNCTIONS
     */
    /**
     * Positive values are from clockwise rotation and negative values are from counter-clockwise
     * @return rotatePower
     */
    public double getRotatePower() {
        double power = joystick.getZ(); 

        //If we are in deadzone or strafelock is on, rotatepower is 0
        if ((Math.abs(power) < 0.3) || (getStrafeLock() == true)) {
            power = 0;
        }

        //Cubes the power and clamps it because the rotate is SUPER sensitive
        power = Math.pow(power, 3.0); 
        power = MathUtil.clamp(power, -.5, .5);
            
        return power;    
    }

    /**
     * Gets the drive X
     * @return driveX
     */
    public double getDriveX() {
        double power = joystick.getX();

        //Strafe lock removes deadzone and cubes power for more precision
        if (getStrafeLock() == true) {
            power = Math.pow(power, 3);
        }
        //If we are in deadzone or rotatelock is on, x is 0
        if ((Math.abs(power) < 0.05) || (getRotateLock() == true)) {
            power = 0;
        }
        return power;
    }

    /**
     * Gets the drive Y
     * @return driveY
     */
    public double getDriveY() {
        double power = joystick.getY() * -1;

        //Strafe lock removes deadzone and cubes power for more precision
        if (getStrafeLock() == true) {
            power = Math.pow(power, 3);
        }
        //If we are in deadzone or rotatelock is on, y is 0
        else if ((Math.abs(power) < 0.05) || (getRotateLock() == true)) {
            power = 0;
        }
        return power;
    }

    /**
     * Checks if we are in strafe lock mode
     * @return joystick button 2
     */
    private boolean getStrafeLock() {
        return joystick.getRawButton(2);
    }

    /**
     * Checks if we are in rotate lock mode
     * @return joystick button 3
     */
    private boolean getRotateLock() {
        return joystick.getRawButton(3);
    }

    /**
     * Returns shoot location based off of trigger and button 4
     * @return shoot location
     */
    public ShootLocation getShootLocation() {
        if (joystick.getTrigger() && joystick.getRawButton(4)) {
            return ShootLocation.LOW_SHOT;
        }
        else if (joystick.getTrigger() && joystick.getRawButton(6)) {
            return ShootLocation.LAUNCH_PAD;
        }
        else if (joystick.getTrigger()) {
            return ShootLocation.HIGH_SHOT;
        }
        else {
            return ShootLocation.OFF;
        }
    }


    /**
     * XBOX CONTROLLER FUNCTIONS
     */
    /**
     * Start Button Pressed
     * <p>KILLS ALL ACTIVE AUTO PROGRAMS!
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
        
    /** 
     * Left Bumper Pressed
     * @return leftBumperPressed 
     */
    public boolean getClimberClaw1() { 
        return xboxController.getLeftBumperPressed(); 
    }

    /**
     * Right Bumper Pressed 
     * @return rightBumperPressed 
     */
    public boolean getClimberClaw2() { 
        return xboxController.getRightBumperPressed(); 
    }

    public double getClimberPower() {
        return -1 * xboxController.getLeftY();
    }
     
    //Grabber Direction based off of D-Pad
    public GrabberDirection getGrabberDirection() {
        if (xboxController.getPOV() == 0) {
            return GrabberDirection.FORWARD;
        }
        else if (xboxController.getPOV() == 180) {
            return GrabberDirection.REVERSE;
        }
        else {
            return GrabberDirection.OFF;
        }
    }

    /**
     * Uses the Xbox Controller's rumbing to display CargoTracking error
     * @param rumblePower
     */
    public void controllerRumble(double rumblePower) {
        double scaledRumble = rumblePower / 240;
        double absPower = Math.abs(scaledRumble);

        if (scaledRumble < 0) {
            xboxController.setRumble(RumbleType.kLeftRumble , absPower / 3);
        }
        else if (scaledRumble > 0) {
            xboxController.setRumble(RumbleType.kRightRumble, absPower / 3);
        }
        else {
            xboxController.setRumble(RumbleType.kLeftRumble , 0);
            xboxController.setRumble(RumbleType.kRightRumble, 0);
        }
    }

    //Test controls
    public double getFeedPower() {
        if (xboxController.getLeftTriggerAxis() > 0) {
            return -0.2 * xboxController.getLeftTriggerAxis();
        }
        else if (xboxController.getRightTriggerAxis() > 0) {
            return 0.2 * xboxController.getRightTriggerAxis();
        }
        else {
            return 0;
        }
    }
}

// End of the Controls class