package frc.robot;

import frc.robot.Grabber.GrabberDirection;

public class Auto {
    
    private int step = 1;

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
                grabber.deployRetract();
                status = Robot.DONE;
                break;
            case 2:
                //Placeholder for limelight targetting and shooting
                status = autoDelay(5000);
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
                //Placeholder for limelight targetting and shooting
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
                //Placeholder for limelight targetting and shooting
                status = autoDelay(5000);
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

    public int autoLimelight() {
        int status = Robot.CONT;
        
        if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch (step) {
            case 1:
                status = autoDelay(5000);
                break;
            case 2:
                status = drive.limelightPIDTargeting();
                break;
            default:
                //
                step = 1;
                firstTime = true;
                System.out.println("Finished Targetting");
                return Robot.DONE;
        }

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
}
