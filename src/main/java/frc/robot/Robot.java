package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Drive.TargetPipeline;
import frc.robot.Shooter.ShootLocation;

/**
 * The class that runs eveything else
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
  Drive                   drive;
  Auto                    auto;
  Grabber                 grabber;
  Shooter                 shooter;
  Climber                 climber;
  Controls                controls;
  CargoTracking           cargoTracking;
  LedLights               led;
  //PowerDistributionPanel  pdp;

  // Variables
  private int status              = Robot.CONT;
  private int targetStatus        = Robot.CONT;
  private boolean lowShotEnabled  = false;

  // Enumeration for manual or automatic control
  public static enum DriveMode {
    MANUAL,
    LIMELIGHT_TARGETING,
    LIMELIGHT_TARGETED,
    CARGO_TARGETING,
    CARGO_TARGETED;
  }
  private DriveMode driveMode = DriveMode.MANUAL;

  // Auto path
  private static final String kCenterAuto = "Center";
  private static final String kWallAuto   = "Wall";
  private static final String kHangarAuto = "Hangar";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Number of balls
  private static final int kTwoBall   = 2;
  private static final int kThreeBall = 3;
  private int m_numBalls;
  private final SendableChooser<Integer> m_numBallsChooser = new SendableChooser<>();

  // Auto Delay
  private int delaySec = 0;

  /**
   * Constructor
   */
  public Robot() {
    // Instance creation
    drive         = new Drive();
    grabber       = new Grabber();
    controls      = new Controls();
    climber       = new Climber();
    shooter       = new Shooter();
    cargoTracking = new CargoTracking(drive);
    auto          = new Auto(drive, grabber, shooter, cargoTracking);
    led           = LedLights.getInstance();
    //pdp           = new PowerDistributionPanel();

    // Creates a Network Tables instance
    FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");

    // Creates the Networktable Entry
    isRedAlliance = FMSInfo.getEntry("IsRedAlliance"); // Boolean
  }

  @Override
  /**
   * robotInit()
   * Runs once when the robot is started
   */
  public void robotInit() {
    // Auto start location
    m_chooser.setDefaultOption("Wall Auto", kWallAuto);
    m_chooser.addOption("Center Auto", kCenterAuto);
    m_chooser.addOption("Hangar Auto", kHangarAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Auto delay
    SmartDashboard.putNumber("Auto delay seconds", 0);

    // Number of Balls to grab
    m_numBallsChooser.setDefaultOption("2 ball", kTwoBall);
    m_numBallsChooser.addOption("3 ball", kThreeBall);
    SmartDashboard.putData("Number of Balls", m_numBallsChooser);

    // Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( getRedAlliance() );
    
    // Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
  }

  @Override
  /**
   * robotPeriodic()
   * Always runs on the robot
   */
  public void robotPeriodic() {
    // Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( getRedAlliance() );

    // Passes our color to the lights
    led.defaultMode( getRedAlliance() );
  }

  @Override
  /**
   * autonomousInit()
   * Runs once when Auto starts
   */
  public void autonomousInit() {
    // Choses start position
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    m_numBalls = m_numBallsChooser.getSelected();
    System.out.println("Auto path: " + m_numBalls);

    delaySec = (int)SmartDashboard.getNumber("Auto delay seconds", 0);

    // Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( getRedAlliance() );

    // Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);

    // Inits the ledlights for auto
    led.autoInit();

    // Resets the gyro
    Drive.ahrs.zeroYaw();
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    int balls = m_numBalls;
    long autoDelayMSec = delaySec * 1000;

    if (status == Robot.CONT) {
      switch (m_autoSelected) {
        case kCenterAuto:
          status = auto.centerAuto(balls, autoDelayMSec);
          break;
        case kHangarAuto:
          status = auto.hangerAuto(balls, autoDelayMSec);
          break;
        case kWallAuto:
          status = auto.wallAuto(balls, autoDelayMSec);
          break;
        default:
          status = DONE;
          break;
      }
    }
  }

  @Override
  /**
   * teleopInit()
   * Runs once at the start of TeleOp
   */
  public void teleopInit() {
    // Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( getRedAlliance() );

    // Inits the led lights
    led.teleopInit();

    // Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
    climber.setClimberIdleMode(IdleMode.kBrake);
  }

  @Override
  /**
   * teleopPeriodic()
   * Runs constantly during TeleOp
   */
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

    // Sets the Led's to team colors when disabled
    led.teamColors();
  }

  @Override
  /**
   * disabledPeriodic()
   * Shouldn't do anything
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
    // Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( getRedAlliance() );

    // Resets status
    status = Robot.CONT;

    // Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
    //shooter.testShootMotor(SmartDashboard.getNumber("Shoot Power", 0.00));
    //drive.testWheelAngle();
  }

  /**
   * Controls the wheels in TeleOp
   */
  private void wheelControl() {
    // Gets Joystick Values
    double driveX               = controls.getDriveX();
    double driveY               = controls.getDriveY();
    double rotatePower          = controls.getRotatePower();
    boolean enableCargoTracking = controls.enableCargoTracking();
    ShootLocation shootLocation = controls.getShootLocation();

    // Kills all automatic funcitons (Start on the Xbox controller)
    boolean autokill            = controls.autoKill();

    if (autokill == true) {
      driveMode = DriveMode.MANUAL;
    } 

    // Manual driving
    if (driveMode == DriveMode.MANUAL) {
      // Drives if we are out of dead zone
      if ((Math.abs(driveX) > 0.05) ||
          (Math.abs(driveY) > 0.05) || 
          (Math.abs(rotatePower) > 0.01)) {
        drive.teleopSwerve(driveX, driveY, rotatePower, false, true);
      }
      else {
        // Robot is in dead zone, doesn't drive
        drive.stopWheels();
      }

      // Sets a variable that is used later in ShooterControl()
      if (shootLocation == ShootLocation.LOW_SHOT) {
        lowShotEnabled = true;
      }
      else {
        lowShotEnabled = false;
      }

      // Exit conditions
      if ((shootLocation == ShootLocation.HIGH_SHOT) || (shootLocation == ShootLocation.LAUNCH_PAD)) {
        driveMode = DriveMode.LIMELIGHT_TARGETING;
      }

      // Exit conditions
      if (enableCargoTracking == true) {
        driveMode = DriveMode.CARGO_TARGETING;
      }
    } 
    // Limelight targeting
    else if (driveMode == DriveMode.LIMELIGHT_TARGETING) {
      if (shootLocation == ShootLocation.OFF || shootLocation == ShootLocation.LOW_SHOT) {
        driveMode = DriveMode.MANUAL;
      }
      else {
        targetStatus = drive.limelightPIDTargeting(shootLocation, Drive.TargetPipeline.ON_TARMAC, true);
      }

      // Exit conditions
      if (targetStatus == Robot.DONE) {
        driveMode = DriveMode.LIMELIGHT_TARGETED;
      }
      else if (targetStatus == Robot.FAIL) {
        driveMode = DriveMode.MANUAL;
      }
    }
    // Limelight targeted
    else if (driveMode == DriveMode.LIMELIGHT_TARGETED) {
      // Does nothing until trigger is released (exit condition)
      if (shootLocation == ShootLocation.OFF || shootLocation == ShootLocation.LOW_SHOT) {
        driveMode = DriveMode.MANUAL;
      }
    }
    // Raspberry Pi Targeting
    else if (driveMode == DriveMode.CARGO_TARGETING) {
      int cargoStatus = cargoTracking.autoCargoTrack();

      if (cargoStatus == Robot.DONE) {
        driveMode = DriveMode.CARGO_TARGETED;
      }
      else if (cargoStatus == Robot.FAIL) {
        driveMode = DriveMode.MANUAL;
      }
    }
    // Raspberry Pi Targeted
    else if (driveMode == DriveMode.CARGO_TARGETED) {
      driveMode = DriveMode.MANUAL;
    }
  }

  /**
   * Controls the ball in TeleOp
   */
  private void ballControl() {
    // Controls 
    boolean deployRetract               = controls.grabberDeployRetract();
    boolean startShooter                = controls.startShooter();
    boolean secureBalls                 = controls.secureBalls();
    boolean releaseBalls                = controls.releaseBalls();
    Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();
    Shooter.ShootLocation shootLocation = controls.getShootLocation();

    // Shooter Ready
    boolean isShooterReady              = shooter.shooterReady();

    /**
     * Grabber control
     */
    if (deployRetract == true) {
      grabber.deployRetract();
    }
    grabber.setGrabberMotor(grabberDir);

    /**
     * Ball blocker
     * <p> Manual controls takes precedent over automatic control
     * <p> Automatically brings the ball blocker up when grabberDeploy() is called
     */
    // Manual
    if ((secureBalls == true) && (releaseBalls == true)) {
      System.out.println("ERROR! BOTH BALL BLOCKER BUTTONS PRESSED!");
    }
    else if (secureBalls == true) {
      grabber.blockBalls();
    }
    else if (releaseBalls == true) {
      grabber.releaseBalls();
    }
    // Automatic
    else if (grabberDir == Grabber.GrabberDirection.INTAKE) {
      grabber.releaseBalls();
    }

    /**
     * Shooter control
     */
    if (shootLocation == Shooter.ShootLocation.OFF) {
      shooter.disableShooter();
      shooter.blockShooter();

      if (startShooter == true) {
        shooter.shooterControl(Shooter.ShootLocation.HIGH_SHOT);
      }
    }
    else {
      shooter.shooterControl(shootLocation);

      if (isShooterReady == true && (lowShotEnabled == true || driveMode == DriveMode.LIMELIGHT_TARGETED)) {
        shooter.openShooter();
        grabber.releaseBalls();
        led.shooterReady();
      }
      else if (isShooterReady == false) {
        shooter.blockShooter();
        led.shooterNotReady();
      }
      else {
        led.shooterNotReady();
      }

      led.updateShooter();
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
  private boolean getRedAlliance() {
    // Gets and returns if we are red from the FMS
    boolean isRed = isRedAlliance.getBoolean(false);
    return isRed;
  }

}
// End of the Robot class