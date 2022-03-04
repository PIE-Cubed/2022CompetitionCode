package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber {
    /*
    Positive power brings right arm (22) yellow claw up from starting point (CCW if looking from right)
    Negative power brings left arm (21) yellow claw up from starting point (CCW if looking from left)
    Encoder for 1st bar: 46.3
    */

    //Spark Max ID for the climber
    private final int CLIMBER_SPARKMAX_ID  = 22;
    private final int CLIMBER_FOLLOW_SPARKMAX_ID = 21;

    //Current limit
    private final int CURRENT_LIMIT = 60;

    //Object creation for the climberMotor
    private CANSparkMax climberMotor; 
    private CANSparkMax climberFollowMotor;
    private RelativeEncoder climberEncoder;

    //Object creation for climberPistons
    private DoubleSolenoid blueClaw;
    private DoubleSolenoid yellowClaw;

    //Double Solenoid for the claws
    private final int PCM_CAN_ID1    = 1; 
    private final int BLUE_CLAW_OPEN    = 3;//7
    private final int BLUE_CLAW_CLOSE   = 7;//3
    private final int YELLOW_CLAW_OPEN  = 2;//6
    private final int YELLOW_CLAW_CLOSE = 6;//2

    //Constants
    private static final double BAR_TWO_POSITION   =  46.30;
    private static final double BAR_THREE_POSITION = -38.50;
    private static final double BAR_FOUR_POSITION  = -120.0;

    //The enum and claw variables
    public enum ClawState {
        OPEN,
        CLOSE;
    }
    private ClawState blueClawState;
    private ClawState yellowClawState;
    
    /**
     * CONSTRUCTOR
     */
    public Climber() {
        //The Motor
        climberMotor = new CANSparkMax(CLIMBER_SPARKMAX_ID, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kCoast);
        climberMotor.setSmartCurrentLimit(CURRENT_LIMIT);

        climberFollowMotor = new CANSparkMax(CLIMBER_FOLLOW_SPARKMAX_ID, MotorType.kBrushless);
        climberFollowMotor.setIdleMode(IdleMode.kCoast);
        climberFollowMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        climberFollowMotor.follow(climberMotor, true);

        climberMotor.set(0.0);

        //Encoder
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0.0); 

        //The Claws
        blueClaw    = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, BLUE_CLAW_OPEN, BLUE_CLAW_CLOSE); 
        yellowClaw  = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, YELLOW_CLAW_OPEN, YELLOW_CLAW_CLOSE); 
        
        blueClawClose();
        yellowClawClose();
        // blueClaw.set(Value.kReverse);
        // yellowClaw.set(Value.kReverse);
        // blueClawState    = ClawState.CLOSE;
        // yellowClawState  = ClawState.CLOSE;
    }

    /**
     * blue claw toggle
     */
    public void blueClawToggle() {
        if (blueClawState == ClawState.OPEN) {
            blueClawClose();
        }
        else if (blueClawState == ClawState.CLOSE) {
            blueClawOpen();
        }
    }

    /**
     * yellow claw toggle
     */
    public void yellowClawToggle() {
        if (yellowClawState == ClawState.OPEN) {
            yellowClawClose();
        }
        else if (yellowClawState == ClawState.CLOSE) {
            yellowClawOpen();
        }
    }

    //Direct setting of pistons
    public void blueClawOpen() {
        blueClaw.set(Value.kForward);
        blueClawState = ClawState.OPEN;
    }
    public void blueClawClose() {
        blueClaw.set(Value.kReverse);
        blueClawState = ClawState.CLOSE;
    }

    public void yellowClawOpen() {
        yellowClaw.set(Value.kForward);
        yellowClawState = ClawState.OPEN;
    }
    public void yellowClawClose() {
        yellowClaw.set(Value.kReverse);
        yellowClawState = ClawState.CLOSE;
    }

    /**
     * Rotates the climber arms
     * @param rotatePower
     * neagtive power raises arm from horizontal start position at bar 1
     */
    public void climberRotate(double rotatePower) {
        climberMotor.set(rotatePower);
        System.out.println("Climber Encoder: " + getClimberEncoder());
    }

    /**
     * encoder values...
     *      horizontal  0
     *      bar1        -15.5
     *      bar2        
     *      bar3        
     * 
     *  how much does encoder change with small changes in rotation?
     * 
     * @return
     */
    public double getClimberEncoder() {
        return climberEncoder.getPosition();
    }

    public int moveToBar2() {
        if (climberEncoder.getPosition() >= BAR_TWO_POSITION) {
            climberMotor.set(0);
            return Robot.DONE;
        }
        else {
            yellowClawOpen();
            blueClawOpen();
            climberMotor.set(0.3);
            return Robot.CONT;
        }
    }

    //Not tested
    public int moveToBar3() {
        if (climberEncoder.getPosition() <= BAR_THREE_POSITION) {
            climberMotor.set(0);
            blueClawClose();
            return Robot.DONE;
        }
        else {
            climberMotor.set(-0.3);
            return Robot.CONT;
        }
    }

    //Not tested
    public int moveToBar4() {
        if (climberEncoder.getPosition() <= BAR_FOUR_POSITION) {
            climberMotor.set(0);
            yellowClawClose();
            return Robot.DONE;
        }
        else {
            climberMotor.set(-0.3);
            return Robot.CONT;
        }
    }

}

// End of the Climber Class