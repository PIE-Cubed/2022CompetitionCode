package frc.robot;

import frc.robot.Grabber.GrabberDirection;

public class Auto {
    
    private int step = 1;
    private int shootStep = 1;

    Drive   drive;
    Grabber grabber;
    Shooter shooter;

    // First Time variables 
	private boolean   firstTime        = true;
	private boolean   shootFirstTime   = true;
	private boolean   delayFirstTime   = true;
    
    private long autoDelayTargetMs = 0;

    /**
     * Constructor
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
            //121 degrees: shooter facing target at start
            //140 degrees: grabber facing ball
            case 1:
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                break;
            case 2:
                status = autoDelay(3000);
                break;
            case 3:
                System.out.println("Targeted");
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 4:
                status = drive.autoRotate(152);
                break;
            case 5:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 6:
                status = drive.autoAdjustWheels(0);
                break;
            case 7:
                status = drive.autoCrabDrive(4, 0, 0.4);
                break;
            case 8:
                status = autoDelay(1000);
                break;
            case 9:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 10:
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.OFF_TARMAC);
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
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                break;
            case 2:
                status = autoDelay(3000);
                break;
            case 3:
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 4:
                status = drive.autoRotate(135);
                break;
            case 5:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 6:
                status = drive.autoAdjustWheels(0);
                break;
            case 7:
                status = drive.autoCrabDrive(4, 0, 0.4);
                break;
            case 8:
                status = autoDelay(1000);
                break;
            case 9:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 10:
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.OFF_TARMAC);
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
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                break;
            case 2:
                status = autoDelay(3000);
                break;
            case 3:
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 4:
                status = drive.autoRotate(90);
                break;
            case 5:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 6:
                status = drive.autoAdjustWheels(0);
                break;
            case 7:
                status = drive.autoCrabDrive(2.5, 0, 0.4);
                break;
            case 8:
                status = autoDelay(1000);
                break;
            case 9:
                grabber.deployRetract();
                grabber.setGrabberMotor(Grabber.GrabberDirection.OFF);
                status = Robot.DONE;
                break;
            case 10:
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.OFF_TARMAC);
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
     * @return
     */
    private int autoShoot() {
        int status = Robot.CONT;

		if (shootFirstTime == true) {
			shootFirstTime = false;
			shootStep = 1;
		}

        switch(shootStep) {
            case 1:
                shooter.manualShooterControl(Shooter.ShootLocation.AUTO_RING);
                drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                status = Robot.DONE;
                break;
            case 2:
                if (shooter.shooterReady()) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
                shooter.manualShooterControl(Shooter.ShootLocation.AUTO_RING);
                drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                break;
            case 3:
                shooter.manualShooterControl(Shooter.ShootLocation.AUTO_RING);
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                break;
            case 4:
                shooter.deployFeeder();
                status = autoDelay(1000);
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