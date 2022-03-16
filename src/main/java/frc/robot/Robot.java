package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.*;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Drive.TargetPipeline;
import frc.robot.Shooter.ShootLocation;

/**
 * The class that runs with the start of a match
 */
public class Robot extends TimedRobot {
  // ERROR CODES
  public static final int FAIL = -1;
  public static final int PASS =  1;
  public static final int DONE =  2;
  public static final int CONT =  3;

  // Networktables
  private NetworkTable FMSInfo;
  private NetworkTableEntry isRedAlliance;	

  // Object creation
  Drive         drive;
  Controls      controls;
  Grabber       grabber;
  Climber       climber;
  Shooter       shooter;
  CargoTracking cargoTracking;
  Auto          auto;
  SwerveDrive   swerveDrive;

  // Variables
  //private int status = Robot.CONT;
  private int targetStatus = Robot.CONT;
  private Command autonomousCommand;

  // CONSTANTS
  private final double DEAD_ZONE = 0.1;

  // Rate limiters
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter turnLimiter;

  // Enumeration for manual or limelight control
  public static enum DriveMode {
    MANUAL,
    LIMELIGHT_TARGETING,
    LIMELIGHT_TARGETED,
    CARGO_TARGETING,
    CARGO_TARGETED;
  }
  private DriveMode driveMode = DriveMode.MANUAL;

  // Enumeration for field drive
  public static enum FieldDrive {
    ENABLED,
    DISABLED;
  }
  private FieldDrive fieldDrive = FieldDrive.DISABLED;

  /**
   * Constructor
   */
  public Robot() {    
    //Instance Creation
    drive         = new Drive();
    grabber       = new Grabber();
    controls      = new Controls();
    climber       = new Climber();
    shooter       = new Shooter();
    cargoTracking = new CargoTracking(drive);
    auto          = new Auto(drive, grabber, shooter, cargoTracking);
    swerveDrive = SwerveDrive.getInstance();

    // Creates rate limiters
    xLimiter = new SlewRateLimiter(.5);
    yLimiter = new SlewRateLimiter(.5);
    turnLimiter = new SlewRateLimiter(.5);

    //Creates a Network Tables instance
    FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");

    //Creates the Networktable Entries
    isRedAlliance = FMSInfo.getEntry("IsRedAlliance"); // Boolean
  }

  @Override
  /**
   * robotInit()
   * Runs once when the robot is started
   */
  public void robotInit() {
    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( setRedAlliance() );
    
    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
  }

  @Override
  /**
   * robotPeriodic()
   * Always runs on the robot
   */
  public void robotPeriodic() {
    // Runs the command scheduler
    CommandScheduler.getInstance().run();
  }

  @Override
  /**
   * autonomousInit()
   * Runs once when Auto starts
   */
  public void autonomousInit() {
    // Get the command we want to run
    autonomousCommand = auto.tragectoryFollow();

    // Schedules the command to run
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( setRedAlliance() );

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);

    Drive.ahrs.zeroYaw();    
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    // Handled in the init funciton
  }

  @Override
  /**
   * teleopInit()
   * Runs once at the start of TeleOp
   */
  public void teleopInit() {
    // Cancels the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( setRedAlliance() );

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
    climber.setClimberIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    wheelControl();
    ballControl();
    climberControl();
  }

  @Override
  /**
   * disabledInit()
   */
  public void disabledInit() {
    // Sets the climber motors to coast
    climber.setClimberIdleMode(IdleMode.kCoast);
  }

  @Override
  /**
   * disabledPeriodic()
   * Shouldn't ever do anything
   */
  public void disabledPeriodic() {
    // Nothing yet...
  }

  @Override
  /**
   * testInit()
   * Runs once at the start of Test
   */
  public void testInit() {
    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( setRedAlliance() );

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
    //
  }

  public void chassisMovement() {
    // Input variables
    double  xSpeed           = controls.getDriveX();
    double  ySpeed           = controls.getDriveY();
    double  turnSpeed        = controls.getRotatePower();
    boolean fieldDriveToggle = controls.toggleFieldDrive();

    // Dead Zone
    xSpeed    = (Math.abs(xSpeed) > DEAD_ZONE)    ? xSpeed : 0.0;
    ySpeed    = (Math.abs(ySpeed) > DEAD_ZONE)    ? ySpeed : 0.0;
    turnSpeed = (Math.abs(turnSpeed) > DEAD_ZONE) ? turnSpeed : 0.0;

    // Makes driving smoother
    xSpeed    = xLimiter   .calculate(xSpeed)    * SwerveModule.MAX_MOVE_SPEED;
    ySpeed    = yLimiter   .calculate(ySpeed)    * SwerveModule.MAX_MOVE_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveModule.MAX_ROTATION_SPEED;

    // Toggles field drive
    if (fieldDriveToggle == true) {
      if (fieldDrive == FieldDrive.DISABLED) {
        fieldDrive = FieldDrive.ENABLED;
      }
      else if (fieldDrive == FieldDrive.ENABLED) {
        fieldDrive = FieldDrive.DISABLED;
      }
      else {
        fieldDrive = FieldDrive.DISABLED;
      }
    }

    // Creates desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldDrive == FieldDrive.ENABLED) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turnSpeed, swerveDrive.getRotation2d());
    }
    else if (fieldDrive == FieldDrive.DISABLED) {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }
    else {
      // Should never occur
      chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = SwerveDrive.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Output each module states to wheels
    swerveDrive.setModuleStates(moduleStates);
}

  /**
   * Controls the wheels in TeleOp
   */
  private void wheelControl() {
    //Gets Joystick Values
    double driveX               = controls.getDriveX();
    double driveY               = controls.getDriveY();
    double rotatePower          = controls.getRotatePower();
    ShootLocation shootLocation = controls.getShootLocation();

    //Gets Xbox Values
    boolean cargoTrackingActive = false;

    //Kills all automatic funcitons (Start on the Xbox controller)
    boolean autokill            = controls.autoKill();

    //General state changes
    if (autokill == true) {
      driveMode = DriveMode.MANUAL;
    } 

    //Manual driving
    if (driveMode == DriveMode.MANUAL) {
      //Drives if we are out of dead zone
      if ((Math.abs(driveX) > 0) ||
          (Math.abs(driveY) > 0) || 
          (Math.abs(rotatePower) > 0)) {
        drive.teleopSwerve(driveX, driveY, rotatePower, false, true);
      }
      else {
        //Robot is in dead zone, doesn't drive
        drive.stopWheels();
      }

      //Exit conditions
      if ((shootLocation == ShootLocation.HIGH_SHOT) || (shootLocation == ShootLocation.LAUNCH_PAD)) {
        driveMode = DriveMode.LIMELIGHT_TARGETING;
      }

      //Exit coditions
      if (cargoTrackingActive == true) {
        driveMode = DriveMode.CARGO_TARGETING;
      }
    } 
    //Limelight targeting
    else if (driveMode == DriveMode.LIMELIGHT_TARGETING) {
      if (shootLocation == ShootLocation.OFF || shootLocation == ShootLocation.LOW_SHOT) {
        driveMode = DriveMode.MANUAL;
      }
      else {
        targetStatus = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
      }

      if (targetStatus == Robot.DONE) {
        driveMode = DriveMode.LIMELIGHT_TARGETED;
      }
      else if (targetStatus == Robot.FAIL) {
        driveMode = DriveMode.MANUAL;
      }
    }
    //Limelight targeted
    else if (driveMode == DriveMode.LIMELIGHT_TARGETED) {
      //Does nothing until trigger is released
      if (shootLocation == ShootLocation.OFF || shootLocation == ShootLocation.LOW_SHOT) {
        driveMode = DriveMode.MANUAL;
      }
    }
    //Raspberry Pi Targeting
    else if (driveMode == DriveMode.CARGO_TARGETING) {
      int    cargoStatus = cargoTracking.autoCargoTrack();
      double error       = cargoTracking.getCenterOffset();

      if (cargoStatus == Robot.CONT) {
        controls.controllerRumble(error);
      }
      else if (cargoStatus == Robot.DONE) {
        driveMode = DriveMode.CARGO_TARGETED;
      }
      else if (cargoStatus == Robot.FAIL) {
        driveMode = DriveMode.MANUAL;
      }
    }
    //Raspberry Pi Targeted
    else if (driveMode == DriveMode.CARGO_TARGETED) {
      driveMode = DriveMode.MANUAL;
    }
  }

  /**
   * Controls the ball in TeleOp
   */
  private void ballControl() {
    /**
     * Grabber control
     */
    boolean deployRetract               = controls.grabberDeployRetract();
    Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();
    Shooter.ShootLocation shootLocation = controls.getShootLocation();

    if (deployRetract == true) {
      grabber.deployRetract();
    }
    grabber.setGrabberMotor(grabberDir);
    
    /**
     * Shooter control
     */
    if (shootLocation == Shooter.ShootLocation.OFF) {
      shooter.disableShooter();
    }
    else {
      shooter.shooterControl(shootLocation);

      if (shooter.shooterReady() == true) {
        shooter.deployFeeder();
      }
    }
  }

  /**
   * Controls the climber in TeleOp
   */
  private void climberControl() {
    // Claw toggles
    boolean toggleBlueClaw   = controls.toggleBlueClaw();
    boolean toggleYellowClaw = controls.toggleYellowClaw();

    // Movement functions
    double  climberPower = controls.getClimberPower();
    boolean moveToBar2   = controls.getClimberMoveToBar2();
    boolean moveToBar3   = controls.getClimberMoveToBar3();
    boolean moveToBar4   = controls.getClimberMoveToBar4();

    // Encoder reset
    boolean resetEncoder = controls.resetClimberEncoder();

    if (toggleBlueClaw == true) {
      climber.blueClawToggle();
    }
    if (toggleYellowClaw == true) {
      climber.yellowClawToggle();
    }

    if (moveToBar2 == true) {
      drive.changePipeline(TargetPipeline.CAMERA);
      climber.moveToBar2();
    }
    else if (moveToBar3 == true) {
      climber.moveToBar3();
    }
    else if (moveToBar4 == true) {
      climber.moveToBar4();
    }
    else {
      climber.climberRotate(climberPower);
    }

    if (resetEncoder == true) {
      climber.resetEncoder();
    }
  }

  /**
   * Determines if we are on the red alliance
   * @return isRed
   */
  private boolean setRedAlliance() {
    //Gets and returns if we are red from the FMS
    boolean isRed = isRedAlliance.getBoolean(false);
    return isRed;
  }

}

//End of the Robot class