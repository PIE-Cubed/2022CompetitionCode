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

    private final boolean useJoystick = true;
    
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
        XBOX_MANIP_CONTROLLER(0),
        LOGITECH_DRIVE_CONTROLLER(1);
        
        private int id;

        // Each item in the enum will now have to be instantiated with a constructor with the integer id. Look few lines above, where JOYSTICK(int id) and XBOXCONTROLLER(int id) are. These are what the constructor is for.
        private ControllerIDs(int id) {
            this.id = id;
        }

        private int getId() {
            return this.id;
        }
    }

    // Joystick Object Declaration
    private Joystick joystick;
    
    // XboxController Object Declaration
    private XboxController xboxController;
    private XboxController driveController;

    private Controls() {
        //Instance Creation
        if (useJoystick) {
            joystick = new Joystick(ControllerIDs.JOYSTICK.getId());

        } else {
            driveController = new XboxController(ControllerIDs.LOGITECH_DRIVE_CONTROLLER.getId());

        }
        xboxController = new XboxController(ControllerIDs.XBOX_MANIP_CONTROLLER.getId());
    }

    /**
     * JOYSTICK METHODS
     */
    public double getX() {
        if (useJoystick) {
            return joystick.getX();
        }
        else {
            return driveController.getRightX();
        }
    }

    public double getY() {
        if (useJoystick) {
            return joystick.getY();
        }
        else {
            return driveController.getLeftY();
        }
    }

    public double getZ() {
        if (useJoystick) {
            return joystick.getZ();
        }
        else {
            return driveController.getRightX();
        }
    }


    private boolean getShooterEnable() {
        if (useJoystick) {
            return joystick.getTrigger();
        }
        else {
            return (driveController.getRightTriggerAxis() > 0.1);
        }
    }

    
     /**
     * 0 degrees is forward on the Joystick
     * this method returns values from -180 to +180
     * @return driveAngle
     */
    public double getDriveAngle() {
        double deadZone = 0.1;

        double x = getX();
        double y = getY();
        
        double rad = Math.atan2(x, y);
        double deg = Math.toDegrees(rad);
        //double = MathUtil.clamp(deg, 0, 360);

        // Drive Power is always positive
        double drivePower = getDrivePower();

        //System.out.println("Dr Pwr " + drivePower + " x " + x + " y " + y);
        
        if ((drivePower < deadZone) && (drivePower > -deadZone)) {
            return 0;
        }
        else {
            return deg;
        }
    }

    /**
     * Positive values are from clockwise rotation 
     * and negative values are from counter-clockwise
     * @return rotatePower
     */
    public double getRotatePower() {
        double deadZone = 0.3;
        double power = getZ();

        if ((power < deadZone) && (power > (deadZone * -1))) {
            //If within the deadzone, don't do anything
            return 0;
        }
        else {
            //Halves the power because the rotate is SUPER sensitive
            power = Math.pow(power, 3.0); 
            power = MathUtil.clamp(power, -.5, .5);
            
            return power;
        }        
    }

    /**
     * Gets the drive power
     * @return drivePower
     */
    public double getDrivePower() {
        double x = getX();
        double y = getY() * -1;
        //double powerMultiplier = powerMultiplier();
        double drivePower;
        double hyp = Math.sqrt(x*x + y*y);
        double hypClamp;

        //This will make it reach power 1 faster at a 45 degrees
        hypClamp = MathUtil.clamp(hyp, -1, 1);
        //hypClamp = hyp / Math.sqrt(2);

        //This makes the throttle actually work, with all the way at the botom being zero throttle
        drivePower = hypClamp; //* powerMultiplier;
        
        return drivePower;
    }

    /**
     * Gets the drive X
     * @return driveX
     */
    public double getDriveX() {
        double power = getX();
        double deadZone = 0.1;

        if ((power < deadZone) && (power > (deadZone * -1))) {
            return 0;
        }
        else {
            //Returns the x-axis power that the robot should drive at
            //return power * powerMultiplier();
            return power;
        }
    }

    /**
     * Gets the drive Y
     * @return driveY
     */
    public double getDriveY() {
        double power = getY() * -1;
        double deadZone = 0.1;

        if ((power < deadZone) && (power > (deadZone * -1))) {
            return 0;
        }
        else {
            //Returns the y-axis power that the robot should drive at
            //return power * powerMultiplier();
            return power;
        }        
    }


    /**
     * Returns the decimal value of the throttle, with all the way at the bottom being 0
     * @return throttleDecimal 0 to 2
     */
    /*
    private double powerMultiplier() {
        //Variables
        double throttle = joystick.getThrottle() * -1; //Forward on the throttle is -1, but we want it to be +1
        double posThrottle;
        double throttleDecimal;

        //Makes the throttle values 0 to 2 (this makes the math easier) 
        posThrottle = throttle + 1;
        //Divides the value by two, getting a decimal
        throttleDecimal = posThrottle / 2;

        return throttleDecimal;
    }*/


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
    public boolean deployRetract() {
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

    //NOT FINAL
    public Grabber.GrabberDirection getGrabberDirection() {
        return Grabber.GrabberDirection.OFF;
    }
    
   /*
    public boolean getFieldDrive() {
        if (joystick.getRawButtonPressed(7)) {
            fieldDrive = false;
            System.out.println("Field drive disabled");
        }
        if (joystick.getRawButtonPressed(8)) {
            fieldDrive = true;
            System.out.println("Field drive enabled");
        }
        return fieldDrive;
    }*/

} // End of the Controls class