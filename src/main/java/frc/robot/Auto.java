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
    
    private long autoDelayTargetMs = 0;


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

            //121 degrees: shooter facing target at start
            //140 degrees: grabber facing ball

            case 1:
                status = autoDelay(5000);
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

    public int autoDelay(int ms) {
        long currentMs = System.currentTimeMillis();

        if (firstTime == true) {
            autoDelayTargetMs = currentMs + ms;
            firstTime = false;
        }

        if (currentMs > autoDelayTargetMs) {
            return Robot.DONE;
        }
        return Robot.CONT;
    }
}
