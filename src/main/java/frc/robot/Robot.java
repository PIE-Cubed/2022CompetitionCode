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
  CargoTracking cargo;

  // ERROR CODES
  public static final int FAIL = -1;
  public static final int PASS =  1;
  public static final int DONE =  2;
  public static final int CONT =  3;

  /**
   * Shuffleboard choices
   */
  //Auto Positions
	private static final String kCustomAutoRight  = "Right";
	private static final String kCustomAutoCenter = "Center";
	private static final String kCustomAutoLeft   = "Left";
	private static final String kCustomAutoLRC    = "L/R/C Simple";
	private String m_positionSelected;
  private final SendableChooser<String> m_pathChooser = new SendableChooser<>();

  //Auto Delay
  private static final String kCustomDelayZero  = "0";
	private static final String kCustomDelayTwo   = "2";
	private static final String kCustomDelayFour  = "4";
	private static final String kCustomDelaySix   = "6";
	private int m_delaySelected;
  private final SendableChooser<String> m_delayChooser = new SendableChooser<>();

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
    //Instance creation
    drive    = new Drive();
    controls = Controls.getInstance();
    cargo    = new CargoTracking(drive);
  }

  @Override
  /**
   * robotInit()
   * Runs once when the robot is started
   */
  public void robotInit() {
    /**
     * Shuffleboard choices
     */
    //Auto Positions
		m_pathChooser.addOption(kCustomAutoRight, kCustomAutoRight);
		m_pathChooser.addOption(kCustomAutoCenter, kCustomAutoCenter);
		m_pathChooser.addOption(kCustomAutoLeft, kCustomAutoLeft);
		m_pathChooser.addOption(kCustomAutoLRC, kCustomAutoLRC);

		//Default Auto Position
		m_pathChooser.setDefaultOption(kCustomAutoLRC, kCustomAutoLRC);
		SmartDashboard.putData("Auto Positions", m_pathChooser);

    //Default Auto Delay
    m_delayChooser.addOption(kCustomDelayZero, kCustomDelayZero);
		m_delayChooser.addOption(kCustomDelayTwo , kCustomDelayTwo);
		m_delayChooser.addOption(kCustomDelayFour, kCustomDelayFour);
    m_delayChooser.addOption(kCustomDelaySix , kCustomDelaySix);
    
    //Default Auto Position
		m_delayChooser.setDefaultOption(kCustomDelayZero, kCustomDelayZero);
		SmartDashboard.putData("Auto Delay", m_delayChooser);

    //Alliance Selection
    m_allianceChooser.addOption(kRedAlliance , kRedAlliance);
		m_allianceChooser.addOption(kBlueAlliance, kBlueAlliance);
    
    //Default Alliance
		m_allianceChooser.setDefaultOption(kDefaultAlliance, kDefaultAlliance);
		SmartDashboard.putData("Alliance Color", m_allianceChooser);
  }

  @Override
  /**
   * robotPeriodic()
   * Always runs on the robot
   * Don't put anything here. It causes a Null Pointer Error
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
    //Set variables
    //
    
    //Auto positions
    m_positionSelected = m_pathChooser.getSelected();

    //Auto Delay
    m_delaySelected = Integer.parseInt(m_delayChooser.getSelected());

    //Alliance
    m_allianceSelected = m_allianceChooser.getSelected();

    //Telemetry
    System.out.println("Delay: "    + m_delaySelected);
		System.out.println("Position: " + m_positionSelected);
    System.out.println("Alliance: " + m_allianceSelected);

    //Passes cargo and alliance color to the Pi for Object Tracking
    cargo.setCargoColor(m_allianceSelected);
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
   */
  public void autonomousPeriodic() {
    //
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
  /**
   * teleopPeriodic()
   * Runs constantly during TeleOp
   */
  public void teleopPeriodic() {
    //Nothing yet...
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
    //Alliance
    m_allianceSelected = m_allianceChooser.getSelected();

    //Telemetry
    System.out.println("Alliance: " + m_allianceSelected);

    //Passes cargo and alliance color to the Pi for Object Tracking
    cargo.setCargoColor(m_allianceSelected);
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
    //
    cargo.faceCargo();
  }


  private void wheelControl() {
    drive.teleopRotate(0.2);

   /* double driveX      = controls.getDriveX();
    double driveY      = controls.getDriveY();
    double rotatePower = controls.getRotatePower();

    if ((Math.sqrt(driveX*driveX + driveY*driveY) > 0.01) || (Math.abs(rotatePower) > 0.01)) {
      drive.teleopSwerve(driveX, driveY, rotatePower, false);
    }
    else {
      //Robot is in dead zone
      drive.stopWheels();
    }*/
  }

/*
  private void ballControl() {
    //Pair of pistons to retract and deploy
    //One motor to take balls in and out
  
    boolean deployRetract = controls.deployRetract();
    Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();


    if(deployRetract == true) {
      grabber.deployRetract();
    }
  }
*/

}
//End of the Robot class