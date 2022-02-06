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

    public Controls() {
        //Instance Creation
        joystick       = new Joystick(ControllerIDs.JOYSTICK.getId());
        xboxController = new XboxController(ControllerIDs.XBOX_MANIP_CONTROLLER.getId());
    }



    // SHOOTER ENABLED
    public boolean getShooterEnable() {        
        return joystick.getTrigger();        
    }

    
    /**
     * 0 degrees is forward on the Joystick
     * this method returns values from -180 to +180
     * @return driveAngle
     */
    public double getDriveAngle() {
        double x = joystick.getX();
        double y = joystick.getY();
        
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
        double power = joystick.getZ(); 
        if (Math.abs(power) < 0.3) {
            power = 0;
        }

        //Cubes the power and clamps it because the rotate is SUPER sensitive
        power = Math.pow(power, 3.0); 
        power = MathUtil.clamp(power, -.5, .5);
            
        return power;    
    }

    /**
     * Gets the drive power
     * @return drivePower
     *
    public double getDrivePower() {
        double x = joystick.getX();
        double y = joystick.getY() * -1;

        //Does math to calculate the power we want if on an angle  
        double hyp = Math.sqrt(x*x + y*y);
        double hypClamp = MathUtil.clamp(hyp, -1, 1);
 
        return hypClamp;
    }*/

    /**
     * Gets the drive X
     * @return driveX
     */
    public double getDriveX() {
        double power = joystick.getX();
        if (Math.abs(power) < 0.05) {
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
        if (Math.abs(power) < 0.05) {
            power = 0;
        }
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
    public Grabber.GrabberDirection getGrabberDirection() {
        if (xboxController.getPOV() == 0) {
            return Grabber.GrabberDirection.FORWARD;
        }
        else if (xboxController.getPOV() == 180) {
            return Grabber.GrabberDirection.REVERSE;
        }
        else {
            return Grabber.GrabberDirection.OFF;
        }
    }
}

// End of the Controls class