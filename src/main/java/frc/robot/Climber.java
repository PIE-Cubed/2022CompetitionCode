package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber {
    //Spark Max ID for the climber
    private final int CLIMBER_SPARKMAX_ID  = 22; 
    private CANSparkMax climberMotor; 

    //Double Solenoid for the claws
    private final int PCM_CAN_ID   = 1; 
    private final int CLAW_1_OPEN  = 1;
    private final int CLAW_1_CLOSE = 5;
    private final int CLAW_2_OPEN  = 2;
    private final int CLAW_2_CLOSE = 6;
    private DoubleSolenoid claw1;
    private DoubleSolenoid claw2;

    //The enum and claw variables
    public enum ClawState {
        OPEN,
        CLOSE;
    }
    private ClawState claw1State;
    private ClawState claw2State;
    
    /**
     * CONSTRUCTOR
     */
    public Climber() {
        //The Motor
        climberMotor  = new CANSparkMax(CLIMBER_SPARKMAX_ID, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(60);
        climberMotor.set(0.0);

        //The Claws
        claw1 = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, CLAW_1_OPEN, CLAW_1_CLOSE); 
        claw2 = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, CLAW_2_OPEN, CLAW_2_CLOSE); 
        claw1.set(Value.kForward);
        claw2.set(Value.kForward);
        claw1State = ClawState.OPEN;
        claw2State = ClawState.OPEN;
    }

    //Toggle The Claws
    public void claw1Toggle() {
        if (claw1State == ClawState.OPEN) {
            claw1.set(Value.kReverse);
            claw1State = ClawState.CLOSE;
        }
        else if (claw1State == ClawState.CLOSE) {
            claw1.set(Value.kForward);
            claw1State = ClawState.OPEN;
        }
    }
    public void claw2Toggle() {
        if (claw2State == ClawState.OPEN) {
            claw2.set(Value.kReverse);
            claw2State = ClawState.CLOSE;
        }
        else if (claw2State == ClawState.CLOSE) {
            claw2.set(Value.kForward);
            claw2State = ClawState.OPEN;
        }
    }

    public void climberRotate(double rotatePower) {
        climberMotor.set(rotatePower);
    }

}

// End of the Climber Class