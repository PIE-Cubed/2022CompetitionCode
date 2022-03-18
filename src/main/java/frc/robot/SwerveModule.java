package frc.robot;

/**
 * Imports
 */
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;

public class SwerveModule {
	// Creates the motors
	private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

	// Creates the encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final AnalogInput rotateMotorSensor;
    private final double absoluteEncoderOffsetRad;

	// Variables

	/**
     * CONSTANTS
     */
    // TICK CONSTANTS
	private final double TICKS_PER_FOOT   = 5.65;
	private final double TICKS_PER_METER  = Units.metersToFeet(TICKS_PER_FOOT);
	private final double TICKS_PER_DEGREE = 7 / 45;
	private final double TICKS_PER_RADIAN = Units.radiansToDegrees((double)TICKS_PER_DEGREE);

    // CONVERSION CONSTANTS
	private final double METERS_PER_SECOND = TICKS_PER_METER / 60;
	private final double RADIANS_PER_SECOND = TICKS_PER_RADIAN / 60;

    // MOVEMENT CONSTANTS
    public static final double MAX_MOVE_SPEED     = 5; // in m/s
    public static final double MAX_ACCELERATION   = 3; // in m/s/s
    public static final double MAX_ROTATION_SPEED = 0.5; // in rad/sec

	/**
     * PID controller
     */
    private final PIDController rotationPID;

	// PID Constants
	private static final double kP = 0.03;
	private static final double kI = 0.00;
	private static final double kD = 0.00;

    public SwerveModule(int driveMotorId, int turningMotorId, int analogId, double absoluteEncoderOffset) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        rotateMotorSensor = new AnalogInput(analogId);

        driveMotor   = new CANSparkMax(driveMotorId  , MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveEncoder   = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(TICKS_PER_METER);
        driveEncoder.setVelocityConversionFactor(METERS_PER_SECOND);
        turningEncoder.setPositionConversionFactor(TICKS_PER_RADIAN);
        turningEncoder.setVelocityConversionFactor(RADIANS_PER_SECOND);

        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = rotateMotorSensor.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d( getTurningPosition() ));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / MAX_MOVE_SPEED);
        turningMotor.set(rotationPID.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + rotateMotorSensor.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}

// End of SwerveModule class