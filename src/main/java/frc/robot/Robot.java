package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of class
 */
public class Robot extends TimedRobot {

  //Object creation
  Drive         drive;
  Controls      controls;
  Grabber       grabber;
  Climber       climber;
  //CargoTracking cargoTracking;
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

  //Alliance
  private static final String kDefaultAlliance = "Default";
  private static final String kRedAlliance = "Red";
  private static final String kBlueAlliance = "Blue";
  private String m_allianceSelected;
  private final SendableChooser<String> m_allianceChooser = new SendableChooser<>();

  /**
   * Constructor
   */
  public Robot() {
    drive         = new Drive();
    grabber       = new Grabber();
    controls      = new Controls();
    climber       = new Climber();
    //cargoTracking = new CargoTracking(drive);
    auto          = new Auto(drive, grabber);
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

    //Alliance Selection
    m_allianceChooser.addOption(kRedAlliance , kRedAlliance);
		m_allianceChooser.addOption(kBlueAlliance, kBlueAlliance);
    
    //Default Alliance
		m_allianceChooser.setDefaultOption(kDefaultAlliance, kDefaultAlliance);
		SmartDashboard.putData("Alliance Color", m_allianceChooser);

    //Sets the limelight LED mode
    drive.limelightEntries.getEntry("ledMode").setNumber(1);
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

    //Get Alliance
    m_allianceSelected = m_allianceChooser.getSelected();

    //Telemetry
    System.out.println("Alliance: " + m_allianceSelected);

    //Passes cargo and alliance color to the Pi for Object Tracking
    //cargoTracking.setCargoColor(m_allianceSelected);
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    if (status == CONT) {
      switch (m_autoSelected) {
        case kCenterAuto:
          status = auto.centerAuto();
          break;
        case kHangarAuto:
          status = DONE;
          break;
        case kWallAuto:
          status = DONE;
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
    //Get Alliance
    m_allianceSelected = m_allianceChooser.getSelected();

    //Telemetry
    System.out.println("Alliance: " + m_allianceSelected);

    //Passes cargo and alliance color to the Pi for Object Tracking
    //cargoTracking.setCargoColor(m_allianceSelected);
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
    //Nothing yet...
  }

  @Override
  /**
   * testInit()
   * Runs once at the start of Test
   */
  public void testInit() {
    //Get Alliance
    m_allianceSelected = m_allianceChooser.getSelected();

    //Telemetry
    System.out.println("Alliance: " + m_allianceSelected);

    //Passes cargo and alliance color to the Pi for Object Tracking
    //cargoTracking.setCargoColor(m_allianceSelected);

    //Sets the limelight LED mode
    drive.limelightEntries.getEntry("ledMode").setNumber(0);
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
    System.out.println(Drive.ahrs.getYaw());
    //drive.testLimelightTargeting();
    //drive.testRotate();
    //drive.testWheelAngle();
  }


  private void wheelControl() {
    //Get joystick values
    double driveX      = controls.getDriveX();
    double driveY      = controls.getDriveY();
    double rotatePower = controls.getRotatePower();

    //Drives if we are out of dead zone
    if ((Math.sqrt(driveX*driveX + driveY*driveY) > 0.01) || (Math.abs(rotatePower) > 0.01)) {
      drive.teleopSwerve(driveX, driveY, rotatePower, false);
    }
    else {
      //Robot is in dead zone, doesn't drive
      drive.stopWheels();
    }
  }


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

  private void climberControl() {
    boolean toggleClaw1  = controls.getClimberClaw1();
    boolean toggleClaw2  = controls.getClimberClaw2();
    double  climberPower = controls.getClimberPower();

    if (toggleClaw1 == true) {
      climber.claw1Toggle();
    }
    if (toggleClaw2 == true) {
      climber.claw2Toggle();
    }
    climber.climberRotate(climberPower);
  }
}
//End of the Robot class