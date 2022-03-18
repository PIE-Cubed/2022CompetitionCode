package frc.robot;

/**
 * Imports
 */
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;

public class SwerveDrive extends SubsystemBase {
	// Singleton because I don't want to deal with passing it in
	private static SwerveDrive instance = null;
	public static synchronized SwerveDrive getInstance() {
		if (instance == null) {
			instance = new SwerveDrive();
		}
		
		return instance;
	}

	// Variables
	
	/**
	 * CONSTANTS
	 */
	// Drive Motor ID's
	private final int FRONT_RIGHT_DRIVE_MOTOR_ID  = 16;
	private final int FRONT_LEFT_DRIVE_MOTOR_ID   = 10;
	private final int BACK_RIGHT_DRIVE_MOTOR_ID   = 14;
	private final int BACK_LEFT_DRIVE_MOTOR_ID    = 12;

	// Rotate Motor ID's
	private final int FRONT_RIGHT_ROTATE_MOTOR_ID = 17;
	private final int FRONT_LEFT_ROTATE_MOTOR_ID  = 11;
	private final int BACK_RIGHT_ROTATE_MOTOR_ID  = 15;
	private final int BACK_LEFT_ROTATE_MOTOR_ID   = 13;

	// Analog Sensor ID's
	private final int FRONT_RIGHT_SENSOR_ID       = 3;
	private final int FRONT_LEFT_SENSOR_ID        = 0;
	private final int BACK_RIGHT_SENSOR_ID        = 2;
	private final int BACK_LEFT_SENSOR_ID         = 1;

	// Analog Sensor Offset (in degrees)
	//BLUE ROBOT
    // private static final double FR_OFFSET =  248;
    // private static final double FL_OFFSET =  309.8;
	// private static final double BR_OFFSET = -33.2;
    // private static final double BL_OFFSET =  165.7;

    //Yellow ROBOT
	private static final double FR_OFFSET = -150.36;
    private static final double FL_OFFSET = -151.21;
	private static final double BR_OFFSET =  146.41; //93.10
    private static final double BL_OFFSET = -103.64;

	// Analog Sensor Offset (in radians)
	private static final double FR_OFFSET_RAD = Math.toRadians(FR_OFFSET);
    private static final double FL_OFFSET_RAD = Math.toRadians(FL_OFFSET);
	private static final double BR_OFFSET_RAD = Math.toRadians(BR_OFFSET);
    private static final double BL_OFFSET_RAD = Math.toRadians(BL_OFFSET);

	// Defines the swerve modules
	private final SwerveModule frontRight = new SwerveModule(
		FRONT_RIGHT_DRIVE_MOTOR_ID,
		FRONT_RIGHT_ROTATE_MOTOR_ID,
		FRONT_RIGHT_SENSOR_ID,
		FR_OFFSET_RAD);
	private final SwerveModule frontLeft = new SwerveModule(
		FRONT_LEFT_DRIVE_MOTOR_ID,
		FRONT_LEFT_ROTATE_MOTOR_ID,
		FRONT_LEFT_SENSOR_ID,
		FL_OFFSET_RAD);
	private final SwerveModule backRight = new SwerveModule(
		BACK_RIGHT_DRIVE_MOTOR_ID,
		BACK_RIGHT_ROTATE_MOTOR_ID,
		BACK_RIGHT_SENSOR_ID,
		BR_OFFSET_RAD);
	private final SwerveModule backLeft = new SwerveModule(
		BACK_LEFT_DRIVE_MOTOR_ID,
		BACK_LEFT_ROTATE_MOTOR_ID,
		BACK_LEFT_SENSOR_ID,
		BL_OFFSET_RAD);

	// NavX Declaration
	public static AHRS ahrs;

	// Robot Dimensions (in inches)
	private static final double ROBOT_LENGTH  = 30.125;
    private static final double ROBOT_WIDTH   = 18.0;

	// Robot Dimensions (in meters)
	private static final double LENGTH_METERS = Units.inchesToMeters(ROBOT_LENGTH);
	private static final double WIDTH_METERS  = Units.inchesToMeters(ROBOT_WIDTH);

	// Swerve Drive kinematics
	public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(WIDTH_METERS / 2, -LENGTH_METERS / 2),
                new Translation2d(WIDTH_METERS / 2, LENGTH_METERS / 2),
                new Translation2d(-WIDTH_METERS / 2, -LENGTH_METERS / 2),
                new Translation2d(-WIDTH_METERS / 2, LENGTH_METERS / 2));

	// Swerve Drive odometry
	private final SwerveDriveOdometry odometer;

	/**
	 * CONSTRUCTOR
	 */
	private SwerveDrive() {
		// NavX
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("Error Instantiating navX MXP: " + ex.getMessage());
        }
    
        ahrs.reset();
    
        while (ahrs.isConnected() == false) {
            System.out.println("Connecting navX");
        }
        System.out.println("navX Connected");
    
        while (ahrs.isCalibrating() == true) {
            System.out.println("Calibrating navX");
        }
        System.out.println("navx Ready");
    
        ahrs.zeroYaw();

		// Creates the SwerveDriveOdometer
		odometer = new SwerveDriveOdometry(driveKinematics, new Rotation2d(0));
	}

	public void zeroHeading() {
		ahrs.reset();
	}

	public double getHeading() {
        return Math.IEEEremainder(ahrs.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees( getHeading() );
    }

	@Override
	/**
	 * Odometer Functions
	 */
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

	public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

	public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

	/**
	 * Swerve Module Functions
	 */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModule.MAX_MOVE_SPEED);
		frontRight.setDesiredState(desiredStates[0]);
		frontLeft.setDesiredState(desiredStates[1]);
		backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

	public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}

// End of SwerveDrive class