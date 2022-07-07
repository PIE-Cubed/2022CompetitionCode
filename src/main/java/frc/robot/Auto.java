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
     * Practice autonomous code for the class
     * @return status
     */
    public int practiceAuto() {
        int status = Robot.CONT;
    
		if (firstTime == true) {
            led.autoMode(); // Led lights turn Aqua

			firstTime = false;
			step = 1;
		}

        switch(step) {
            //CODE GOES HERE
            //CHECK THE DOCUMENT IN THIS FOLDER FOR A LIST OF USEFUL FUNCTIONS

            case 1:
                //Format for function that runs over multiple iterations and returns a status
                status = autoDelay(1000); //autoDelay() returns CONT. until it has been ran for a given amount of milliseconds, where it returns DONE
                break;
            case 2:
                //Format for functions that run over one iteration and don't return a status
                grabber.grabberDeploy(); //grabberDeploy() brings down the grabber. This only runs once and doesn't need to return a status
                grabber.setGrabberMotor(GrabberDirection.INTAKE);
                status = Robot.DONE; //In order for the code to go to the next step, we must manually set status to DONE
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.grabberDeploy();
                grabber.releaseBalls();
                shooter.disableShooter();
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
                grabber.grabberDeploy();
                grabber.setGrabberMotor(GrabberDirection.INTAKE);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                status = drive.autoCrabDrive(3.5, 0, 0.2); //DO NOT ADJUST!
                break;
            case 5:
                status = drive.autoAdjustWheels(180);
                break;
            case 6:
                shooter.shooterControl(ShootLocation.HIGH_SHOT);
                status = drive.autoCrabDrive(1.50, 180, 0.2);
                break;
            case 7:
                status = autoShoot(ShootLocation.HIGH_SHOT, 4);
                break;
            case 8:
                status = drive.autoCrabDrive(4.0, 0);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.grabberDeploy();
                grabber.releaseBalls();
                shooter.disableShooter();
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
                grabber.grabberDeploy();
                grabber.setGrabberMotor(GrabberDirection.INTAKE);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                status = drive.autoCrabDrive(3.0, 0, 0.2); //DO NOT ADJUST!
                break;
            case 5:
                status = drive.autoAdjustWheels(180);
                break;
            case 6:
                shooter.shooterControl(ShootLocation.HIGH_SHOT);
                status = drive.autoCrabDrive(1.25, 180);
                break;
            case 7:
                status = autoShoot(ShootLocation.HIGH_SHOT, 4);
                break;
            case 8:
                status = drive.autoCrabDrive(4.0, 0);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.grabberDeploy();
                grabber.releaseBalls();
                shooter.disableShooter();
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
                grabber.grabberDeploy();
                grabber.setGrabberMotor(GrabberDirection.INTAKE);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                status = drive.autoCrabDrive(3.0, 0, 0.2); //DO NOT ADJUST!
                break;
            case 5:
                status = drive.autoAdjustWheels(180);
                break;
            case 6:
                shooter.shooterControl(ShootLocation.HIGH_SHOT);
                status = drive.autoCrabDrive(1.25, 180, 0.2);
                break;
            case 7:
                status = autoShoot(ShootLocation.HIGH_SHOT, 4);
                break;
            // case 8:
            //     step = 12;
            //     status = Robot.DONE;
            //     break;
            // case 9:
            //     status = drive.autoRotate(50);
            //     break;
            // case 10:
            //     status = drive.autoCrabDrive(7.0, 90, 0.5);
            //     break;
            // case 11:
            //     grabber.setGrabberMotor(GrabberDirection.INTAKE);
            //     status = drive.autoCrabDrive(2.15, 0, 0.2);
            //     break;
            // case 12:
            //     status = autoShoot(ShootLocation.AUTO_RING, 2);
            //     break;
            case 8:
                grabber.grabberRetract();
                status = Robot.DONE;
                break;
            case 9:
                status = autoDelay(1000);
                break;
            case 10:
                status = drive.autoCrabDrive(2.5, 0);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                //Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.grabberDeploy();
                shooter.disableShooter();
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
    public int autoShoot(ShootLocation location, int numBalls) {
        int status = Robot.CONT;
        TargetPipeline targetingLocation;

        boolean isShooterReady = shooter.shooterReady();

        if ( (location == ShootLocation.HIGH_SHOT) || (location == ShootLocation.AUTO_RING) ) {
            targetingLocation = TargetPipeline.ON_TARMAC;
        }
        else {
            targetingLocation = TargetPipeline.OFF_TARMAC;
        }

		if (shootFirstTime == true) {
            isShooterReady = false;
			shootFirstTime = false;
			shootStep = 1;
		}

        switch(shootStep) {
            case 1:
                // Starts speeding up shooter and targeting
                shooter.shooterControl(location);
                drive.limelightPIDTargeting(location, targetingLocation, false);
                led.autoMode(); // Led lights turn Aqua
                status = Robot.DONE;
                break;
            case 2:
                // Starts speeding up shooter and targeting
                shooter.shooterControl(location);
                status = drive.limelightPIDTargeting(location, targetingLocation, false);
                break;
            case 3:
                // Continues speeding up shooter and targeting until shooter is at correct RPM
                shooter.shooterControl(location);
                if (isShooterReady == true) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
                break;
            case 4:
                // Retracts the shooter pistons to fire balls
                shooter.openShooter();
                grabber.releaseBalls();
                shooter.shooterControl(location);
                status = autoDelay(numBalls * 1000);
                break;
            default:
                // Finished routine
                shooter.disableShooter();
                shootStep = 1;
                shootFirstTime = true;

                // Led lights turn Gold
                led.autoModeFinished();

                // Returns error codes for success
                return Robot.DONE;
        }

        // If we are done with a step, we go on to the next one and continue the routine
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
                grabber.grabberDeploy();
                status = Robot.DONE;
                break;
            case 3:
                System.out.println("Yaw: " + Drive.ahrs.getYaw());
                grabber.setGrabberMotor(GrabberDirection.INTAKE);
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
                grabber.grabberRetract();
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
            grabber.grabberRetract();
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

    public void resetFirstTime() {
        delayFirstTime = true;
    }
}

// End of Auto Class