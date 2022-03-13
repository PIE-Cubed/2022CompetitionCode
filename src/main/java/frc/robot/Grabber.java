package frc.robot;

/**
 * Imports
 */
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * Start of class
 */
public class Grabber {
    //SPARK MAX ID's
    private static final int SPARK_ID  = 18;

    //PNEUMATICS IDS
    private final int PCM_CAN_ID    = 1;
    private final int DEPLOY_ID     = 4;
    private final int RETRACT_ID    = 0;

    //SPARK MAX CURRENT LIMIT
    private int GRABBER_CURRENT_LIMIT = 60;

    //Spark Max Motors
    private CANSparkMax grabberMotor;

    //Pistons
    private DoubleSolenoid grabberPiston;

    //CONSTANTS
    private final double GRABBER_POWER = -0.7;

    /**
     * Enumerator for Grabber States
     */
    public static enum GrabberState {
        DEPLOY,
        RETRACT;
    }
    private GrabberState grabberState;

    /**
     * Enumerater for Grabber Direction
     */
    public static enum GrabberDirection {
        FORWARD,
        REVERSE,
        OFF;
    }

    /**
     * CONSTRUCTOR
     */
    public Grabber()  {
        //Grabber Motor Init
        grabberMotor = new CANSparkMax(SPARK_ID, MotorType.kBrushed);
        grabberMotor.setSmartCurrentLimit(GRABBER_CURRENT_LIMIT);
        grabberMotor.set(0.0);

        //Grabber Piston Init
        grabberPiston = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, DEPLOY_ID, RETRACT_ID);
        grabberPiston.set(Value.kReverse);
        grabberState = GrabberState.RETRACT;
    }

    /*
     * EXTEND / RETRACT FORWARDING PISTON
     */
    public void deployRetract() {
        // Toggle the State of the Piston
        if (grabberState == GrabberState.DEPLOY) {
            retract();
        }
        else if (grabberState == GrabberState.RETRACT) {
            deploy();
        }
    }

    public void setGrabberMotor(GrabberDirection dir) {
        // Grabber Intake
        if (dir == GrabberDirection.FORWARD) {
            grabberMotor.set(GRABBER_POWER);
        }
        // Grabber Reverse
        else if (dir == GrabberDirection.REVERSE) {
            grabberMotor.set(GRABBER_POWER * -1);
        }
        // No direction
        else {
            grabberMotor.set(0.0);
        }
    }

    /**
     * Individual functions
     */
    public void deploy() {
        grabberPiston.set(Value.kForward);
        grabberState = GrabberState.DEPLOY;
    }

    public void retract() {
        grabberPiston.set(Value.kReverse);
        grabberState = GrabberState.RETRACT;
    }

}

// End of the Grabber Class