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
    public Auto(Drive drive, Grabber grabber, Shooter shooter, CargoTracking cargoTracking){
        this.drive         = drive;
		this.grabber       = grabber;
        this.shooter       = shooter;
        this.cargoTracking = cargoTracking;
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
                grabber.deployRetract();
                //Leave grabber motor on while shooting to help loose balls get in
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 2:
                status = drive.autoAdjustWheels(0);
                break;
            case 3:
                status = autoDelay(delayMs);
                break;                
            case 4:
                status = drive.autoCrabDrive(3.5, 0, 0.4);
                break;
            case 5:
                status = autoDelay(250);
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
                status = drive.autoRotate(-44);
                break;
            case 9:
                status = drive.autoAdjustWheels(-90);
                break;
            case 10:
                status = drive.autoCrabDrive(7, -90, 0.7);
                break;
            case 11:
                status = Robot.DONE;//drive.autoAdjustWheels(0);
                break;
            case 12:
                status = drive.autoCrabDrive(1.5, 0, 0.5);
                break;
            case 13:
                //Delay for balls to calm down
                status = Robot.DONE;//autoDelay(1000);
                break;
            case 14:
                //grabber.setGrabberMotor(GrabberDirection.OFF);
                //grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 15:
                status = Robot.DONE;//autoDelay(750);
                break;
            case 16:
                status = drive.autoRotate(-45); 
                break;
            case 17:
                status = autoShoot(ShootLocation.AUTO_RING, 1);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;
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
                status = autoDelay(delayMs);
                break;
            case 2:
                status = drive.autoRotate(-130);
                break;
            case 3:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 4:
                status = drive.autoAdjustWheels(0);
                break;
            case 5:
                status = drive.autoCrabDrive(4, 0, 0.25);
                break;
            case 6:
                status = autoDelay(1500);
                break;
            case 7:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 8:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
                break;
            case 9:
                //If we are only doing 1 ball path, the routine is over
                if (balls < 3) {
                    step = 100;
                }
                status = Robot.DONE;
                break;
            case 10:
                status = drive.autoRotate(90);
                break;
            case 11:
                status = drive.autoAdjustWheels(0);
                break;
            case 12:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                break;
            case 13:
                status = drive.autoCrabDrive(8, 0);
                break;
            case 14:
                status = autoDelay(1000);
                break;
            case 15:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 16:
                status = drive.autoRotate(-45);
                break;
            case 17:
                status = autoShoot(ShootLocation.AUTO_RING, 1);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;
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
     * @return status
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
                grabber.setGrabberMotor(GrabberDirection.OFF);
                grabber.retract();
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
                status = drive.autoRotate(110);
                break;
            case 9:
                status = drive.autoAdjustWheels(0);
                break;
            case 10:
                grabber.deploy();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 11:
                status = drive.autoCrabDrive(5.6, 0, 0.5);
                break;
            case 12:
                status = drive.autoRotate(45);
                break;
            case 13:
                status = autoShoot(ShootLocation.AUTO_RING, 1);
                break;
            default:
                //Finished routine
                step = 1;
                firstTime = true;

                //Stops applicable motors
                grabber.setGrabberMotor(GrabberDirection.OFF);
                drive.stopWheels();

                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }

        //System.out.println("Case: " + step);

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
                shooter.shooterControl(location);
                drive.limelightPIDTargeting(targettingLocation);
                status = Robot.DONE;
                break;
            case 2:
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
                shooter.shooterControl(location);
                status = drive.limelightPIDTargeting(targettingLocation);
                break;
            case 4:
                shooter.deployFeeder();
                status = autoDelay(500);
                break;
            case 5:
                if (numBalls == 2) {
                    shooter.retractFeeder();
                    status = autoDelay(1000);
                }
                else {
                    status = Robot.DONE;
                }
                break;
            case 6:
                if (numBalls == 2) {
                    shooter.deployFeeder();
                    status = autoDelay(500);
                }
                else {
                    status = Robot.DONE;
                }
                break;                 
            default:
                //Finished routine
                shooter.disableShooter();
                shootStep = 1;
                shootFirstTime = true;
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

    /****************************************************************************************** 
    *
    *    autoInit()
    *    Resets auto variables
    * 
    ******************************************************************************************/
    public void autoInit() {
        // Step Variables
        step = 1;
        shootStep = 1;
        cargoStep = 1;

        // First Time variables 
        firstTime      = true;
        shootFirstTime = true;
        cargoFirstTime = true;
        delayFirstTime = true;
    }
}

// End of Auto Class