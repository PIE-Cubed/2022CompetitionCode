package frc.robot;

import frc.robot.Grabber.GrabberDirection;
import frc.robot.Shooter.ShootLocation;
import frc.robot.Drive.TargetPipeline;

public class Auto {
    // Step Variables
    private int step = 1;
    private int shootStep = 1;

    // Object creation
    Drive   drive;
    Grabber grabber;
    Shooter shooter;

    // First Time variables 
	private boolean   firstTime        = true;
	private boolean   shootFirstTime   = true;
	private boolean   delayFirstTime   = true;
    
    private long autoDelayTargetMs = 0;

    /**
     * CONSTRUCTOR
     * @param drive
     * @param grabber
     */
    public Auto(Drive drive, Grabber grabber, Shooter shooter){
        this.drive   = drive;
		this.grabber = grabber;
        this.shooter = shooter;
    }

    /**
     * Autonomous program for the position closest to the center
     * @return status
     */
    public int centerAuto() {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoRotate(152);
                break;
            case 2:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                status = drive.autoCrabDrive(4, 0, 0.4);
                break;
            case 5:
                status = autoDelay(1000);
                break;
            case 6:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 7:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
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
    public int hangerAuto() {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoRotate(135);
                break;
            case 2:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                status = drive.autoCrabDrive(4, 0, 0.4);
                break;
            case 5:
                status = autoDelay(1000);
                break;
            case 6:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 7:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
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
    public int wallAuto() {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoRotate(90);
                break;
            case 2:
                grabber.deployRetract();
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoAdjustWheels(0);
                break;
            case 4:
                status = drive.autoCrabDrive(2.5, 0, 0.4);
                break;
            case 5:
                status = autoDelay(1000);
                break;
            case 6:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 7:
                status = autoShoot(ShootLocation.AUTO_RING, 2);
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
    private int autoShoot(ShootLocation location) {
        return autoShoot(location, 1);
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
                    status = autoDelay(500);
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
        }

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