package frc.robot;

import frc.robot.Grabber.GrabberDirection;

public class Auto {
    
    private int step = 1;
    private int shootStep = 1;

    Drive   drive;
    Grabber grabber;

    // First Time variables 
	private boolean   firstTime        = true;
	private boolean   shootFirstTime   = true;
	private boolean   routineFirstTime = true;
	private boolean   delayFirstTime   = true;
    
    private long autoDelayTargetMs = 0;

    /**
     * Constructor
     * @param drive
     * @param grabber
     */
    public Auto(Drive drive, Grabber grabber){
        this.drive    = drive;
		this.grabber  = grabber;
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
                drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                status = autoDelay(2500);
                break;
            case 2:
                System.out.println("Targeted");
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoRotate(152);
                break;
            case 4:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 5:
                status = drive.autoAdjustWheels(0);
                break;
            case 6:
                status = drive.autoCrabDrive(4, 0, 0.4);
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
                //Placeholder for limelight targeting and shooting
                status = autoDelay(5000);
                break;
            case 2:
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoRotate(152);
                break;
            case 4:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 5:
                status = drive.autoAdjustWheels(0);
                break;
            case 6:
                status = drive.autoCrabDrive(4, 0, 0.4);
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
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 3:
                status = drive.autoRotate(90);
                break;
            case 4:
                grabber.setGrabberMotor(GrabberDirection.FORWARD);
                status = Robot.DONE;
                break;
            case 5:
                status = drive.autoAdjustWheels(0);
                break;
            case 6:
                status = drive.autoCrabDrive(2.5, 0, 0.4);
                break;
            case 7:
                status = autoDelay(1000);
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

    private int autoShoot() {
        int status = Robot.CONT;

		if (shootFirstTime == true) {
			shootFirstTime = false;
			shootStep = 1;
		}

        switch(shootStep) {
            case 1:
                drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                //shooter.manualShooterControl(ShootLocation.HIGH_SHOT);
                status = Robot.DONE;
                break;
            case 2:
                //if (shooter.shooterReady()) {
                //    status = Robot.DONE;
                //}
                //else {
                //    status = Robot.CONT;
                //}
                //shooter.manualShooterControl(ShootLocation.HIGH_SHOT);
                drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                status = Robot.DONE;
                break;
            case 3:
                //shooter.manualShooterControl(ShootLocation.HIGH_SHOT);
                status = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
                break;
            case 4:
                status = autoDelay(1000);
                //shooter.manualBallFeederControl(Shooter.BallFeederDirection.FORWARD);
                break;
            default:
                //Finished routine
                //shooter.disableBallFeeder();
                //shooter.disableShooter();
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
}
