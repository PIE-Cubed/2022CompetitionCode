package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber {
    //Spark Max ID for the climber
    private final int CLIMBER_SPARKMAX_ID  = 22;

    //Object creation for the climberMotor
    private CANSparkMax climberMotor; 
    private RelativeEncoder climberEncoder;

    //Double Solenoid for the claws
    private final int PCM_CAN_ID1   = 1; 
    private final int CLIMBER_LOCK = 5;
    private final int BLUE_CLAW_OPEN    = 7;
    private final int BLUE_CLAW_CLOSE   = 3;
    private final int YELLOW_CLAW_OPEN  = 2;
    private final int YELLOW_CLAW_CLOSE = 6;
    private DoubleSolenoid blueClaw;
    private DoubleSolenoid yellowClaw;
    private Solenoid climberLock;

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
        climberMotor.setSmartCurrentLimit(60);
        climberMotor.set(0.0);

        //Encoder
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0.0); 

        //The Claws
        blueClaw   = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, BLUE_CLAW_OPEN, BLUE_CLAW_CLOSE); 
        yellowClaw = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, YELLOW_CLAW_OPEN, YELLOW_CLAW_CLOSE); 
        climberLock = new Solenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, CLIMBER_LOCK);
        
        blueClaw.set(Value.kForward);
        yellowClaw.set(Value.kForward);
        climberLock.set(true);
        blueClawState    = ClawState.OPEN;
        yellowClawState  = ClawState.OPEN;
        climberLockState = ClawState.OPEN;
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
            yellowClaw.set(Value.kReverse);
            yellowClawState = ClawState.CLOSE;
        }
        else if (yellowClawState == ClawState.CLOSE) {
            yellowClaw.set(Value.kForward);
            yellowClawState = ClawState.OPEN;
        }
    }

    /**
     * Climber lock toggle
     */
    public void climberLockToggle() {
        if (climberLockState == ClawState.OPEN) {
            climberLock.set(true);
            climberLockState = ClawState.CLOSE;
        }
        else if (climberLockState == ClawState.CLOSE) {
            climberLock.set(false);
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
        climberLock.set(true);
        climberLockState = ClawState.OPEN;
    }
    public void climberLockRetract() {
        climberLock.set(false);
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

}

// End of the Climber Class