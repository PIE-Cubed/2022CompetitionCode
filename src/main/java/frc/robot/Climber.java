package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber {
    //Spark Max ID for the climber
    private final int CLIMBER_SPARKMAX_ID  = 22;

    //Current limit
    private final int CURRENT_LIMIT = 60;

    //Object creation for the climberMotor
    private CANSparkMax climberMotor; 
    private RelativeEncoder climberEncoder;

    //Object creation for climberPistons
    private DoubleSolenoid blueClaw;
    private DoubleSolenoid yellowClaw;
    private DoubleSolenoid climberLock;

    //Double Solenoid for the claws
    private final int PCM_CAN_ID1    = 1; 
    private final int CLIMBER_LOCK   = 5;
    private final int CLIMBER_UNLOCK = 1;
    private final int BLUE_CLAW_OPEN    = 7;
    private final int BLUE_CLAW_CLOSE   = 3;
    private final int YELLOW_CLAW_OPEN  = 2;
    private final int YELLOW_CLAW_CLOSE = 6;

    //The enum and claw variables
    public enum ClawState {
        OPEN,
        CLOSE;
    }
    private ClawState blueClawState;
    private ClawState yellowClawState;
    private ClawState climberLockState;
    
    /**
     * CONSTRUCTOR
     */
    public Climber() {
        //The Motor
        climberMotor  = new CANSparkMax(CLIMBER_SPARKMAX_ID, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        climberMotor.set(0.0);

        //Encoder
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0.0); 

        //The Claws
        blueClaw    = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, BLUE_CLAW_OPEN, BLUE_CLAW_CLOSE); 
        yellowClaw  = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, YELLOW_CLAW_OPEN, YELLOW_CLAW_CLOSE); 
        climberLock = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, CLIMBER_LOCK, CLIMBER_UNLOCK);
        
        blueClaw.set(Value.kForward);
        yellowClaw.set(Value.kForward);
        climberLock.set(Value.kForward);
        blueClawState    = ClawState.OPEN;
        yellowClawState  = ClawState.OPEN;
        climberLockState = ClawState.CLOSE;
    }

    /**
     * Claw one toggle
     */
    public void blueClawToggle() {
        if (blueClawState == ClawState.OPEN) {
            blueClaw.set(Value.kReverse);
            blueClawState = ClawState.CLOSE;
        }
        else if (blueClawState == ClawState.CLOSE) {
            blueClaw.set(Value.kForward);
            blueClawState = ClawState.OPEN;
        }
    }

    /**
     * Claw two toggle
     */
    public void yellowClawToggle() {
        if (yellowClawState == ClawState.OPEN) {
            yellowClaw.set(Value.kForward);
            yellowClawState = ClawState.CLOSE;
        }
        else if (yellowClawState == ClawState.CLOSE) {
            yellowClaw.set(Value.kReverse);
            yellowClawState = ClawState.OPEN;
        }
    }

    /**
     * Climber lock toggle
     */
    public void climberLockToggle() {
        if (climberLockState == ClawState.OPEN) {
            climberLock.set(Value.kReverse);
            climberLockState = ClawState.CLOSE;
        }
        else if (climberLockState == ClawState.CLOSE) {
            climberLock.set(Value.kForward);
            climberLockState = ClawState.OPEN;
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

    public void climberLockDeploy() {
        climberLock.set(Value.kForward);
        climberLockState = ClawState.OPEN;
    }
    public void climberLockRetract() {
        climberLock.set(Value.kReverse);
        climberLockState = ClawState.CLOSE;
    }

    /**
     * Rotates the climber arms
     * @param rotatePower
     * neagtive power raises arm from horizontal start position at bar 1
     */
    public void climberRotate(double rotatePower) {
        climberMotor.set(rotatePower);
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

    public int moveToBar3() {
        if (climberEncoder.getPosition() <= -15.5) {
            climberMotor.set(0);
            blueClawClose();
            return Robot.DONE;
        }
        else {
            climberMotor.set(0.2);
            return Robot.CONT;
        }
    }

}

// End of the Climber Class