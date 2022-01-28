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
  Drive     drive;
  Controls  controls;
  Grabber   grabber;

  // ERROR CODES
  public static final int FAIL = -1;
  public static final int PASS =  1;
  public static final int DONE =  2;
  public static final int CONT =  3;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * Constructor
   */
  public Robot() {
    drive    = new Drive();
    grabber  = new Grabber();
    controls = Controls.getInstance();
  }

  @Override
  /**
   * robotInit()
   * Runs once when the robot is started
   */
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  /**
   * teleopInit()
   * Runs once at the start of TeleOp
   */
  public void teleopInit() {
    //Nothing yet...
  }

  @Override
  public void teleopPeriodic() {
    wheelControl();
    ballControl();
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
    //Nothing yet...
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
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
  
    boolean deployRetract = controls.grabberDeployRetract();
    Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();

    if (deployRetract == true) {
      grabber.deployRetract();
    }

    grabber.setGrabberMotor(grabberDir);
  }
}
//End of the Robot class