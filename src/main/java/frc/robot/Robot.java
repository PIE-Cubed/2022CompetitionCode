package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Variables
  private int status = Robot.CONT;
  private int targetStatus = 0;

  //Enumeration for manual or limelight control
  public static enum DriveMode {
    MANUAL,
    LIMELIGHT_TARGETING,
    LIMELIGHT_TARGETED,
    CARGO_TARGETING,
    CARGO_TARGETED;
  }
  private DriveMode driveMode = DriveMode.MANUAL;

  //Auto path
  private static final String kCenterAuto = "Center";
  private static final String kWallAuto   = "Wall";
  private static final String kHangarAuto = "Hangar";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Number of balls
  private static final int kOneBall = 1;
  private static final int kTwoBall = 2;
  private int m_numBalls;
  private final SendableChooser<Integer> m_numBallsChooser = new SendableChooser<>();

  //Auto Delay
  private int delaySec = 0;

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
    //Auto selection
    m_chooser.setDefaultOption("Center Auto", kCenterAuto);
    m_chooser.addOption("Wall Auto", kWallAuto);
    m_chooser.addOption("Hangar Auto", kHangarAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("Auto delay seconds", 0);

    //Number of Balls to grab
    m_numBallsChooser.setDefaultOption("1 ball", kOneBall);
    m_numBallsChooser.addOption("2 ball", kTwoBall);
    SmartDashboard.putData("Number of Balls", m_numBallsChooser);

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
    //Nothing yet...
  }

  @Override
  /**
   * autonomousInit()
   * Runs once when Auto starts
   */
  public void autonomousInit() {
    //Choses start position
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    m_numBalls = m_numBallsChooser.getSelected();
    System.out.println("Auto path: " + m_numBalls);

    delaySec = (int)SmartDashboard.getNumber("Auto delay seconds", 0);

    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( setRedAlliance() );

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
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
    else if (status == DONE) {
      //
    }
  }

  @Override
  /**
   * teleopInit()
   * Runs once at the start of TeleOp
   */
  public void teleopInit() {
    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance( setRedAlliance() );

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
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
   * Shouldn't ever do anything
   */
  public void disabledInit() {
    //Nothing yet...
  }

  @Override
  /**
   * disabledPeriodic()
   * Shouldn't ever do anything
   */
  public void disabledPeriodic() {
    //Turns off the limelight LEDs when the robot is disabled, saves our eyes
    //drive.changeledMode(Drive.LEDState.OFF);
  }

  @Override
  /**
   * testInit()
   * Runs once at the start of Test
   */
  public void testInit() {
    //Passes if we are on the red alliance to the Pi for Object Tracking
    SmartDashboard.putNumber("Shooter power", 0.55);
    //SmartDashboard.putNumber("Climber power", 0);

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
    //cargoTracking.autoCargoTrack();
    // System.out.println("Climber encoder: " + climber.getClimberEncoder());
    shooter.autoShooterControl(ShootLocation.AUTO_RING);
    //shooter.testShootMotors(SmartDashboard.getNumber("Shooter power", 0));
    //drive.testWheelAngle();
    if (controls.grabberDeployRetract()) {
      shooter.deployFeeder();
    }
    if (controls.testButtonB()) {
      shooter.retractFeeder();
    }
    /*
    shooter.autoShooterControl(ShootLocation.LOW_SHOT);
    SmartDashboard.putBoolean("Shooter ready", shooter.shooterReady());
    SmartDashboard.putNumber("Test front rpm", shooter.getabsRPM(19));
    SmartDashboard.putNumber("Test rear rpm" , shooter.getabsRPM(20));
    SmartDashboard.putNumber("Target RPM", 1450);
    SmartDashboard.putNumber("80% RPM", 1450 * 0.8);*/
    //System.out.println(shooter.testFlipperSwitch());
    //shooter.powerFeeder(controls.getFeedPower());
    //drive.testLimelightTargeting();
    //drive.testRotate();
    //shooter.testShooter(.60);
    //drive.testWheelAngle();
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
        drive.teleopSwerve(driveX, driveY, rotatePower, false);
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
        if (shootLocation == ShootLocation.LAUNCH_PAD) {
          targetStatus = drive.limelightPIDTargeting(Drive.TargetPipeline.OFF_TARMAC);
        }
        else {
          targetStatus = drive.limelightPIDTargeting(Drive.TargetPipeline.ON_TARMAC);
        }
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
    /*
      Grabber control
     */
    boolean deployRetract               = controls.grabberDeployRetract();
    Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();
    Shooter.ShootLocation shootLocation = controls.getShootLocation();

    if (deployRetract == true) {
      grabber.deployRetract();
    }
    grabber.setGrabberMotor(grabberDir);

    /*
      Shooter control
     */
    if (shootLocation == Shooter.ShootLocation.OFF) {
      shooter.disableShooter();
    }
    else {
      shooter.autoShooterControl(shootLocation);
      //System.out.println("Shooter On");

      if (shooter.shooterReady() == true) {
        System.out.println("Shooter Ready");
        shooter.deployFeeder();
      }
    }
  }

  /**
   * Controls the climber in TeleOp
   */
  private void climberControl() {
    boolean toggleBlueClaw   = controls.toggleBlueClaw();
    boolean toggleYellowClaw = controls.toggleYellowClaw();
    boolean openClimberLock  = controls.getClimberLock();
    double  climberPower     = controls.getClimberPower();
    
    if (toggleBlueClaw == true) {
      climber.blueClawToggle();
    }
    if (toggleYellowClaw == true) {
      climber.yellowClawToggle();
    }
    if (openClimberLock == true) {
    //  climber.climberLockRetract();
        climber.climberLockToggle();
    }
    climber.climberRotate(climberPower);

    if (controls.getClimberMoveToBar3()) {
      climber.moveToBar3();
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