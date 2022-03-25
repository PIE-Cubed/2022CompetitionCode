package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;
import frc.robot.Drive.TargetPipeline;
import frc.robot.Grabber.GrabberDirection;
import frc.robot.Shooter.ShootLocation;

public class Auto {
    // Object creation
    Drive   drive;
    Grabber grabber;
    Shooter shooter;
    CargoTracking cargoTracking;
    LedLights led;

    // Step Variables
    private int step = 1;
    private int shootStep = 1;
    private int cargoStep = 1;

    // First Time variables 
	private boolean firstTime      = true;
	private boolean shootFirstTime = true;
    private boolean cargoFirstTime = true;
	private boolean delayFirstTime = true;
    
    // Variables
    private long   autoDelayTargetMs = 0;
    //private int    noTargetCount     = 0;
    private double cargoHeading      = 0;

    /**
     * CONSTRUCTOR
     * @param drive
     * @param grabber
     */
    public Auto(Drive drive, Grabber grabber, Shooter shooter, CargoTracking cargoTracking) {
        // Instance creation
        this.drive         = drive;
		this.grabber       = grabber;
        this.shooter       = shooter;
        this.cargoTracking = cargoTracking;
        led                = LedLights.getInstance();
    }

    /**
     * Autonomous program for the position closest to the center
     * @param balls
     * @param delayMs
     * @return status
     */
    public int centerAuto(int balls, long delayMs) {
        int status = Robot.CONT;

        // Ensures that the values aren't too high
        if (balls > 3 || balls < 2) {
            balls = 2;
        }

        // Ensures that the values aren't too high
        delayMs = (long)MathUtil.clamp((double)delayMs, 0, 6000);
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                Drive.ahrs.zeroYaw();
                status = autoDelay(delayMs);
                led.autoMode(); // Led lights turn Aqua
                break;
            case 2:
                grabber.deploy();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                shooter.shooterControl(ShootLocation.AUTO_RING);
                status = drive.autoCrabDrive(3.5, 0, 0.2); //DO NOT ADJUST!
                break;
            case 5:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.deploy();
                shooter.disableShooter();
                shooter.retractFeeder();
                drive.stopWheels();
 
                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }
        
        return Robot.CONT;
    }

    /**
     * Autonomous program for the position closest to the hanger
     * @param balls
     * @param delayMs
     * @return status
     */
    public int hangerAuto(int balls, long delayMs) {
        int status = Robot.CONT;

        // Ensures that the values aren't too high
        if (balls > 3 || balls < 2) {
            balls = 2;
        }

        // Ensures that the values aren't too high
        delayMs = (long)MathUtil.clamp((double)delayMs, 0, 6000);

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                Drive.ahrs.zeroYaw();
                status = autoDelay(delayMs);
                led.autoMode(); // Led lights turn Aqua
                break;
            case 2:
                grabber.deploy();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                shooter.shooterControl(ShootLocation.AUTO_RING);
                status = drive.autoCrabDrive(3.0, 0, 0.2); //DO NOT ADJUST!
                break;
            case 5:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.deploy();
                shooter.disableShooter();
                shooter.retractFeeder();
                drive.stopWheels();

                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * Autonomous program for the position closest to the wall
     * @param balls
     * @param delayMs
     * @return
     */
    public int wallAuto(int balls, long delayMs) {
        int status = Robot.CONT;

        // Ensures that the values aren't too high
        if (balls > 3 || balls < 2) {
            balls = 2;
        }

        // Ensures that the values aren't too high
        delayMs = (long)MathUtil.clamp((double)delayMs, 0, 6000);

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                Drive.ahrs.zeroYaw();
                status = autoDelay(delayMs);
                led.autoMode(); // Led lights turn Aqua
                break;
            case 2:
                grabber.deploy();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                shooter.shooterControl(ShootLocation.AUTO_RING);
                status = drive.autoCrabDrive(3.0, 0, 0.2); //DO NOT ADJUST!
                break;
            case 5:
                grabber.retract();
                grabber.setGrabberMotor(GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 6:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
                break;
            case 7:
                //If we are only doing 2 ball path, the routine is over
                if (balls < 3) {
                    step = 100;
                }
                status = Robot.DONE;
                break;
            case 8:
                status = drive.autoRotate(50);
                break;
            case 9:
                grabber.deploy();
                status = drive.autoCrabDrive(7.0, 90, 0.5);
                break;
            case 10:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = drive.autoCrabDrive(2.25, 0, 0.2);
                break;
            case 11:
                status = autoShoot(ShootLocation.AUTO_RING, 1);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                //Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.deploy();
                shooter.disableShooter();
                shooter.retractFeeder();
                drive.stopWheels();

                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * An auto program to shoot balls
     * @param location
     * @param numBalls
     * @return status
     */
    private int autoShoot(ShootLocation location, int numBalls) {
        int status = Robot.CONT;
        TargetPipeline targettingLocation;

        if ( (location == ShootLocation.HIGH_SHOT) || (location == ShootLocation.AUTO_RING) ) {
            targettingLocation = TargetPipeline.ON_TARMAC;
        }
        else {
            targettingLocation = TargetPipeline.OFF_TARMAC;
        }

		if (shootFirstTime == true) {
			shootFirstTime = false;
			shootStep = 1;
		}

        switch(shootStep) {
            case 1:
                //Starts speeding up shooter and targetting
                shooter.shooterControl(location);
                drive.limelightPIDTargeting(targettingLocation);
                led.autoMode(); // Led lights turn Aqua
                status = Robot.DONE;
                break;
            case 2:
                //Continues speeding up shooter and targetting until shooter is at correct RPM
                if (shooter.shooterReady()) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
                shooter.shooterControl(location);
                drive.limelightPIDTargeting(targettingLocation);
                break;
            case 3:
                //Deploys feeder for 0.5 seconds
                shooter.deployFeeder();
                shooter.shooterControl(location);
                status = autoDelay(500);
                break;
            case 4:
                //If we are shooting 2 balls, retract feeder and wait 0.5 seconds
                if (numBalls == 2) {
                    shooter.retractFeeder();
                    shooter.shooterControl(location);
                    status = autoDelay(500);
                }
                else {
                    step = 100;
                    status = Robot.DONE;
                }
                break;
            case 5:
                //Once the shooter is ready, move on
                shooter.shooterControl(location);
                if (shooter.shooterReady()) {
                    status = Robot.DONE;
                }
                break;
            case 6:
                //Deploys final ball
                shooter.deployFeeder();
                shooter.shooterControl(location);
                status = autoDelay(500);
                break;                 
            default:
                //Finished routine
                shooter.disableShooter();
                shootStep = 1;
                shootFirstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Returns error codes for success
                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            shootStep++;
        }

        return Robot.CONT;
    }

    /**
     * A method to autmatically track and pick up balls
     * @return status
     */
    public int autoCargoPickup() {
        int status = Robot.CONT;

        // Runs the firstTime procedure
        if (cargoFirstTime == true) {
			cargoFirstTime = false;
			cargoStep = 1;
		}

        switch (cargoStep) {
            case 1:
                status = cargoTracking.autoCargoTrack();
                led.autoMode(); // Led lights turn Aqua
                break;
            case 2:
                grabber.deploy();
                status = Robot.DONE;
                break;
            case 3:
                System.out.println("Yaw: " + Drive.ahrs.getYaw());
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                cargoHeading = Drive.ahrs.getYaw();
                status = Robot.DONE;
                break;
            case 4:
                status = drive.autoAdjustWheels(0.00);
                break;
            case 5:
                status = drive.autoCrabDrive(2.00, cargoHeading, 0.10);
                break;
            default:
                // Finishes the routine
                cargoStep = 1;
                cargoFirstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Resets applicable motors
                grabber.retract();
                grabber.setGrabberMotor(GrabberDirection.OFF);

                // Returns the error code for success
                return Robot.DONE;
        }

        // If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            cargoStep++;
        }

        // If a step fails, exits the routine
        if (status == Robot.FAIL) {
            // Resets variables
            cargoStep = 1;
            cargoFirstTime = true;

            // Stops applicable motors
            grabber.retract();
            grabber.setGrabberMotor(GrabberDirection.OFF);

            // Returns the error code for failure 
            return Robot.FAIL;
        }

        // Returns the error code for continue
        return Robot.CONT;
    }

    /**
     * A program to return a value after a certain number of miliseconds has passed 
     * @param miliseconds
     * @return status
     */
    public int autoDelay(long ms) {
        long currentMs = System.currentTimeMillis();

        if (delayFirstTime == true) {
            autoDelayTargetMs = currentMs + ms;
            delayFirstTime = false;
        }

        if (currentMs > autoDelayTargetMs) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    public void resetFirstTIme() {
        delayFirstTime = true;
    }
}

// End of Auto Class