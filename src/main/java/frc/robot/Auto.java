package frc.robot;

public class Auto {
    
    private int step = 1;

    Drive   drive;
    Grabber grabber;

    // First Time variables 
	private boolean   firstTime        = true;
	private boolean   shootFirstTime   = true;
	private boolean   routineFirstTime = true;
	private boolean   delayFirstTime   = true;

    public Auto(Drive drive, Grabber grabber){
        this.drive    = drive;
		this.grabber  = grabber;
    }

    public int centerAuto() {
        int status = Robot.CONT;

		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoCrabDrive(4, 0, 0.2);
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

}
