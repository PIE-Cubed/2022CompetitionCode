package frc.robot;

/**
 * Imports
 */
import frc.robot.Grabber.GrabberDirection;
import frc.robot.Shooter.ShootLocation;
import frc.robot.Drive.TargetPipeline;

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
    private double cargoHeading      = 0;

    /**
     * CONSTRUCTOR
     * @param drive
     * @param grabber
     */
    public Auto(Drive drive, Grabber grabber, Shooter shooter, CargoTracking cargoTracking){
        this.drive   = drive;
		this.grabber = grabber;
        this.shooter = shooter;
        this.cargoTracking = cargoTracking;
    }

    /**
     * Autonomous program for the position closest to the center
     * @return status
     */
    public int centerAuto(int balls, long delayMs) {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = autoDelay(delayMs);
            case 2:
                status = drive.autoRotate(152);
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
                status = drive.autoCrabDrive(4, 0, 0.4);
                break;
            case 6:
                status = autoDelay(1000);
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
                if (balls == 1) {
                    step = 100;
                }
                status = Robot.DONE;
                break;            
            case 10:
                status = drive.autoRotate(35);
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
                status = drive.autoRotate(-90);
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
     * @return
     */
    public int hangerAuto(int balls, long delayMs) {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = autoDelay(delayMs);
            case 2:
                status = drive.autoRotate(135);
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
                status = drive.autoCrabDrive(4, 0, 0.4);
                break;
            case 6:
                status = autoDelay(1000);
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
                if (balls == 1) {
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
     * @return
     */
    public int wallAuto(int balls, long delayMs) {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = autoDelay(delayMs);
            case 2:
                status = drive.autoRotate(90);
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
                status = drive.autoCrabDrive(2.5, 0, 0.4);
                break;
            case 6:
                status = autoDelay(1000);
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
                if (balls == 1) {
                    step = 100;
                }
                status = Robot.DONE;
                break;
            case 10:
                status = drive.autoRotate(-140);
                break;
            case 11:
                status = drive.autoAdjustWheels(0);
                break;
            case 12:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.FORWARD); 
                status = Robot.DONE;
                break;
            case 13:
                status = drive.autoCrabDrive(8, 0);
                break;
            case 14:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 15:
                status = drive.autoRotate(-45);
                break;
            case 16:
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
     * An auto program to shoot balls
     * @param location
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
                shooter.manualShooterControl(location);
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
                shooter.manualShooterControl(location);
                drive.limelightPIDTargeting(targettingLocation);
                break;
            case 3:
                shooter.manualShooterControl(location);
                status = drive.limelightPIDTargeting(targettingLocation);
                break;
            case 4:
                shooter.deployFeeder();
                status = autoDelay(1000);
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
                    status = autoDelay(1000);
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
            shootFirstTime = true;
        }

        return Robot.CONT;
    }

    public int testPoints() {
        int status = Robot.CONT;

        if (firstTime == true) {
            step = 1;
            firstTime = false;
        }

        switch(step) {
            case 1:
                Drive.ahrs.zeroYaw();
                grabber.deploy();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
            case 2:
                status = drive.goToPoint(-4, 0, -90);
                break;
            case 3:
                status = autoDelay(2000);
                break;
            default:
                //Finished routine
                firstTime = true;
                step = 1;
                drive.stopWheels();
                return Robot.DONE;
        }
        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

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
                shootStep = 1;
                shootFirstTime = true;

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
            shootStep = 1;
            shootFirstTime = true;

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
     * @return
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
}

// End of Auto Class