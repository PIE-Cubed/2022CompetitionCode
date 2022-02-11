package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of class
 */
public class Robot extends TimedRobot {
  //Networktables
  private NetworkTable FMSInfo;
  private NetworkTableEntry isRedAlliance;	

  //Object creation
  Drive         drive;
  Controls      controls;
  Grabber       grabber;
  //Climber       climber;
  Shooter       shooter;
  CargoTracking cargoTracking;
  Auto          auto;

  // ERROR CODES
  public static final int FAIL = -1;
  public static final int PASS =  1;
  public static final int DONE =  2;
  public static final int CONT =  3;

  private int status = CONT;

  //Auto path
  private static final String kCenterAuto = "Center";
  private static final String kWallAuto   = "Wall";
  private static final String kHangarAuto = "Hangar";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * Constructor
   */
  public Robot() {
    //Instance Creation
    drive         = new Drive();
    grabber       = new Grabber();
    controls      = new Controls();
    //climber       = new Climber();
    shooter       = new Shooter();
    cargoTracking = new CargoTracking(drive);
    auto          = new Auto(drive, grabber);

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

    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance(setRedAlliance());
    
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance(setRedAlliance());

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    if (status == Robot.CONT) {
      switch (m_autoSelected) {
        case kCenterAuto:
          status = auto.centerAuto();
          break;
        case kHangarAuto:
          status = auto.hangerAuto();
          break;
        case kWallAuto:
          status = auto.wallAuto();
          break;
        default:
          status = DONE;
          break;
      }
    }
    else if (status == DONE) {

    }
  }

  @Override
  /**
   * teleopInit()
   * Runs once at the start of TeleOp
   */
  public void teleopInit() {
    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance(setRedAlliance());

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
    //Turns off the limelight LEDs when the robot is disabled
    drive.changeledMode(Drive.LEDState.OFF);
  }

  @Override
  /**
   * testInit()
   * Runs once at the start of Test
   */
  public void testInit() {
    //Passes if we are on the red alliance to the Pi for Object Tracking
    cargoTracking.setRedAlliance(setRedAlliance());

    //Sets the limelight LED mode
    drive.changeledMode(Drive.LEDState.ON);
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
    //grabber.setGrabberMotor(Grabber.GrabberDirection.FORWARD);
    //drive.testLimelightTargeting();
    //drive.testRotate();
    //drive.testWheelAngle();
    //cargoTracking.faceCargo();
    shooter.testShooter(.60);
  }

  /**
   * Controls the wheels in TeleOp
   */
  private void wheelControl() {
    //Get joystick values
    double driveX      = controls.getDriveX();
    double driveY      = controls.getDriveY();
    double rotatePower = controls.getRotatePower();

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
  }

  /**
   * Controls the grabber in TeleOp
   */
  private void ballControl() {
    //Connected pair of pistons to retract and deploy
    //One motor to take balls in and out
  
    boolean deployRetract               = controls.grabberDeployRetract();
    Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();

    if (deployRetract == true) {
      grabber.deployRetract();
    }

    grabber.setGrabberMotor(grabberDir);
  }

  /**
   * Controls the climber in TeleOp
   */
  private void climberControl() {
    boolean toggleClaw1  = controls.getClimberClaw1();
    boolean toggleClaw2  = controls.getClimberClaw2();
    double  climberPower = controls.getClimberPower();

    /*
    if (toggleClaw1 == true) {
      climber.claw1Toggle();
    }
    if (toggleClaw2 == true) {
      climber.claw2Toggle();
    }
    climber.climberRotate(climberPower);*/
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