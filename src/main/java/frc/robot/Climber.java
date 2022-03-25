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

    //Variables
    private boolean barFourFirstTime = true;

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
    private final int PCM_CAN_ID1       = 1; 
    private final int BLUE_CLAW_OPEN    = 3;
    private final int BLUE_CLAW_CLOSE   = 7;
    private final int YELLOW_CLAW_OPEN  = 2;
    private final int YELLOW_CLAW_CLOSE = 6;

    //Constants
    private static final double BAR_TWO_POSITION   =  46.30;
    private static final double BAR_THREE_POSITION = -57.50;
    private static final double BAR_FOUR_POSITION  = -210.39;

    //The enum and claw variables
    public enum ClawState {
        OPEN,
        CLOSE;
    }
    private ClawState blueClawState;
    private ClawState yellowClawState;

    // Object creation
    private LedLights led;
    
    /**
     * CONSTRUCTOR
     */
    public Climber() {
        //The Motor
        climberMotor = new CANSparkMax(CLIMBER_SPARKMAX_ID, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(CURRENT_LIMIT);

        climberFollowMotor = new CANSparkMax(CLIMBER_FOLLOW_SPARKMAX_ID, MotorType.kBrushless);
        climberFollowMotor.setIdleMode(IdleMode.kBrake);
        climberFollowMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        climberFollowMotor.follow(climberMotor, true);

        climberMotor.set(0.0);

        //Encoder
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0.0); 

        //The Claws
        blueClaw    = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, BLUE_CLAW_OPEN, BLUE_CLAW_CLOSE);
        yellowClaw  = new DoubleSolenoid(PCM_CAN_ID1, PneumaticsModuleType.CTREPCM, YELLOW_CLAW_OPEN, YELLOW_CLAW_CLOSE);

        // Instance creation
        led = LedLights.getInstance();

        // Closes the claws
        blueClawClose();
        yellowClawClose();
    }

    /**
     * Blue claw toggle
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
     * Yellow claw toggle
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
     * Moves to the bar two position automatically
     * @return atPosition
     */
    public int moveToBar2() {
        if (getClimberEncoder() >= BAR_TWO_POSITION) {
            climberMotor.set(0);

            return Robot.DONE;
        }
        else {
            yellowClawOpen();
            blueClawOpen();
            
            if (getClimberEncoder() >= 0.8 * BAR_TWO_POSITION) {
                climberMotor.set(0.2);
            }
            else {
                climberMotor.set(0.4);
            }
            led.climberMoving();

            return Robot.CONT;
        }
    }

    /**
     * Moves to the bar three position automatically
     * @return atPosition
     */
    public int moveToBar3() {
        if (getClimberEncoder() <= BAR_THREE_POSITION) {
            climberMotor.set(0);
            blueClawClose();

            return Robot.DONE;
        }
        else {
            climberMotor.set(-0.5);
            led.climberMoving();

            return Robot.CONT;
        }
    }

    /**
     * Moves to the bar four position automatically
     * @return atPosition
     */
    public int moveToBar4() {
        if (getClimberEncoder() <= BAR_FOUR_POSITION) {
            climberMotor.set(0);
            //yellowClawClose();
            led.climberDone();

            return Robot.DONE;
        }
        else {
            if (getClimberEncoder() <= 0.85 * BAR_FOUR_POSITION) {
                if (barFourFirstTime == true) {
                    yellowClawOpen();
                    barFourFirstTime = false;
                }

                climberMotor.set(-0.3);
            }
            else {
                barFourFirstTime = true;
                yellowClawClose();
                climberMotor.set(-0.5);
            }
            led.climberMoving();

            return Robot.CONT;
        }
    }

    /**
     * TEST FUNCTIONS
     */
    /**
     * Rotates the main climber motor
     * @param rotatePower
     * <p>Positive raises motor 22
     * <p>Positive lowers motor 21
     * <p>Positive power raises arm from horizontal start position at bar 1
     */
    public void climberRotate(double rotatePower) {
        climberMotor.set(rotatePower);
    }

    /**
     * Rotates the following climber motor (you have to remove it from follow though)
     * @param rotatePower
     * <p>Positive raises motor 22
     * <p>Positive lowers motor 21
     * <p>Positive power raises arm from horizontal start position at bar 1
     */
    public void climberFollowerRotate(double rotatePower) {
        climberFollowMotor.set(rotatePower);
    }

    /**
     * Sets the climber mode (brake or coast)
     * @param value
     */
    public void setClimberIdleMode(IdleMode value) {
        climberMotor.setIdleMode(value);
        climberFollowMotor.setIdleMode(value);
    }

    /**
     * Gets the Encoder Value
     * @return encoderValue
     */
    public double getClimberEncoder() {
        return climberEncoder.getPosition();
    }

    /**
     * Resets the climber encoder
     */
    public void resetEncoder() {
        System.out.println("Resetting climber encoders");
        climberEncoder.setPosition(0.00);
    }

}

// End of the Climber Class