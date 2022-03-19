package frc.robot;

/**
 * Imports
 */
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.stream.DoubleStream;

/**
 * Start of class
 */
public class Drive {
    //Network Table creation
    private NetworkTable limelightEntries;

    //Network Table Entries
    private NetworkTableEntry targetX;
    private NetworkTableEntry targetY;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry targetValid;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry cameraMode;
    private NetworkTableEntry stream;
    private NetworkTableEntry pipeline;

    //NAVX
    public static AHRS ahrs;

    //PID controllers
    private PIDController rotateController;
    private PIDController autoCrabDriveController;
    private PIDController targetController;

    private static final double rotateToleranceDegrees = 2.0f;
    private static final double kLimeLightToleranceDegrees = 3.0f;
    
    // Turn Controller
	private static final double kP = 0.01; //0.02
	private static final double kI = 0.00;
    private static final double kD = 0.00;
    
    //Auto crab drive controller
    private static final double acdP = 0.02; //0.03
    private static final double acdI = 0.00;
    private static final double acdD = 0.00;

	//Target Controller
	private static final double tP = 0.015; //0.033
	private static final double tI = 0.00;
    private static final double tD = 0.00;

	//Variables
    private boolean firstTime               = true;
    private boolean rotateFirstTime         = true;
    private int     count                   = 0;
    private double  encoderTarget           = 0;
    private double  targetOrientation       = 0;
    private double  previousXPower          = 0;
    private double  previousYPower          = 0;
    private double  previousRotatePower     = 0;
    
    //CONSTANTS
    private final int    FAIL_DELAY   = 5;
    private final double ticksPerFoot = 5.65;

    //BLUE ROBOT
    // private static final double FL_OFFSET = 309.8;
    // private static final double FR_OFFSET = 248;
    // private static final double BL_OFFSET = 165.7;
    // private static final double BR_OFFSET = -65.2;

    //Yellow ROBOT
    private static final double FL_OFFSET = -151.28;
    private static final double FR_OFFSET = -150.27;
    private static final double BL_OFFSET = -105.96;
    private static final double BR_OFFSET =  126.93;

	//Limelight Variables
    private int     noTargetCount      = 0;
    private int     targetLockedCount  = 0;
    private long    timeOut;
    private boolean limeLightFirstTime = true;
	private static final int ON_TARGET_COUNT = 5;
    private static final int ON_ANGLE_COUNT  = 10;

    //Limelight
	public boolean limeControl   = false;
	public int     limeStatus    = 0;

        
    /**
     * Enumerators
     */
    /**
     * The enumerator for locking the drive wheels for targeting
     */
    public static enum WheelMode {
		MANUAL,
        TRACKING,
        LOCKED;
    }
    
    /**
     * The enumerator for choosing a target location
     */
    public static enum TargetPipeline {
		ON_TARMAC,
        OFF_TARMAC;
	}

    /**
     * The enumerator for LED state
     */
    public static enum LEDState {
        ON,
        OFF;
    }

    /**
     * An enum containing each wheel's properties including: drive and rotate motor IDs, drive motor types, and rotate sensor IDs
     */ 
    public enum WheelProperties {
        FRONT_RIGHT_WHEEL(16, // DRIVE MOTOR ID
                        17, // ROTATE MOTOR ID
                        3, // ROTATE SENSOR ID
                        (-1 * rotateMotorAngleRad), // ROTATE MOTOR TARGET ANGLE (IN RADIANS)
                        FR_OFFSET), //Offset
        FRONT_LEFT_WHEEL(10, // DRIVE MOTOR ID
                        11, // ROTATE MOTOR ID
                        0, // ROTATE SENSOR ID
                        (-1 * rotateMotorAngleRad - (Math.PI/2)), // ROTATE MOTOR TARGET ANGLE (IN RADIANS)
                        FL_OFFSET), //Offset
        REAR_RIGHT_WHEEL(14, // DRIVE MOTOR ID
                        15, // ROTATE MOTOR ID
                        2, // ROTATE SENSOR ID
                        (-1 * rotateMotorAngleRad + (Math.PI/2)), // ROTATE MOTOR TARGET ANGLE (IN RADIANS)
                        BR_OFFSET), //Offset
        REAR_LEFT_WHEEL(12, // DRIVE MOTOR ID
                        13, // ROTATE MOTOR ID
                        1, // ROTATE SENSOR ID
                        (-1 * rotateMotorAngleRad + (Math.PI)), // ROTATE MOTOR TARGET ANGLE (IN RADIANS)
                        BL_OFFSET); //Offset

        private int    driveMotorId;
        private int    rotateMotorId;
        private int    rotateSensorId;
        private double offsetDegrees; //Inverse of the reading when wheel is physically at 0 degrees

        // Each item in the enum will now have to be instantiated with a constructor with the all of the ids and the motor type constants. Look few lines above, where FRONT_RIGHT_WHEEL(int driveMotorId, MotorType driveMotorType, int rotateMotorId, int rotateSensorId, double targetRadians, double targetVoltage), REAR_LEFT_WHEEL(int driveMotorId, MotorType driveMotorType, int rotateMotorId, int rotateSensorId, double targetRadians, double targetVoltage), etc... are. These are what the constructor is for.
        private WheelProperties(int driveMotorId, int rotateMotorId, int rotateSensorId, double targetRadians, double offsetDegrees) {
            this.driveMotorId = driveMotorId;
            this.rotateMotorId = rotateMotorId;
            this.rotateSensorId = rotateSensorId;
            this.offsetDegrees = offsetDegrees;
        }

        private int getDriveMotorId() {
            return this.driveMotorId;
        }

        private int getRotateMotorId() {
            return this.rotateMotorId;
        }

        private int getRotateSensorId() {
            return this.rotateSensorId;
        }

        private double getOffsetDegrees(){
            return this.offsetDegrees;
        }
    }

    private Wheel frontRightWheel = new Wheel(WheelProperties.FRONT_RIGHT_WHEEL.getDriveMotorId(),
                                              WheelProperties.FRONT_RIGHT_WHEEL.getRotateMotorId(), 
                                              WheelProperties.FRONT_RIGHT_WHEEL.getRotateSensorId(),
                                              WheelProperties.FRONT_RIGHT_WHEEL.getOffsetDegrees(),
                                              WheelProperties.FRONT_RIGHT_WHEEL);
    private Wheel frontLeftWheel  = new Wheel(WheelProperties.FRONT_LEFT_WHEEL.getDriveMotorId(), 
                                              WheelProperties.FRONT_LEFT_WHEEL.getRotateMotorId(), 
                                              WheelProperties.FRONT_LEFT_WHEEL.getRotateSensorId(),
                                              WheelProperties.FRONT_LEFT_WHEEL.getOffsetDegrees(),
                                              WheelProperties.FRONT_LEFT_WHEEL);
    private Wheel rearRightWheel  = new Wheel(WheelProperties.REAR_RIGHT_WHEEL.getDriveMotorId(), 
                                              WheelProperties.REAR_RIGHT_WHEEL.getRotateMotorId(), 
                                              WheelProperties.REAR_RIGHT_WHEEL.getRotateSensorId(),
                                              WheelProperties.REAR_RIGHT_WHEEL.getOffsetDegrees(),
                                              WheelProperties.REAR_RIGHT_WHEEL);
    private Wheel rearLeftWheel   = new Wheel(WheelProperties.REAR_LEFT_WHEEL.getDriveMotorId(), 
                                              WheelProperties.REAR_LEFT_WHEEL.getRotateMotorId(), 
                                              WheelProperties.REAR_LEFT_WHEEL.getRotateSensorId(),
                                              WheelProperties.REAR_LEFT_WHEEL.getOffsetDegrees(),
                                              WheelProperties.REAR_LEFT_WHEEL);
    
    /**
     * The literal lengths and widths of the robot. Look to the swerve drive Google Doc
     * Note: these fields are static because they must be. They are referenced in the enum, which is in and of itself, static.
     * These measurements are in inches
     */
    private static final double robotLength = 30.125;
    private static final double robotWidth  = 18.0;
    private static final double rotateMotorAngleRad = Math.atan2(robotLength, robotWidth);
    private static final double rotateMotorAngleDeg = Math.toDegrees(rotateMotorAngleRad);
 
    // These numbers were selected to make the angles between -180 and +180
    private static final double rotateRightFrontMotorAngle = -1 * rotateMotorAngleDeg; //-1 * rotateMotorAngleDeg;
    private static final double rotateLeftFrontMotorAngle = -180 + rotateMotorAngleDeg; //rotateRightFrontMotorAngle - 90;
    private static final double rotateRightRearMotorAngle = rotateMotorAngleDeg; //rotateRightFrontMotorAngle + 90;
    private static final double rotateLeftRearMotorAngle =  180 - rotateMotorAngleDeg;       //rotateRightFrontMotorAngle + 180;


    /****************************************************************************************** 
    *
    *    PowerAndAngle class
    *    Stores angle and power instead of x and y values
    * 
    ******************************************************************************************/
    public class PowerAndAngle{
        public double power;
        public double angle;

        /****************************************************************************************** 
        *
        *    PowerAndAngle constructor
        * 
        ******************************************************************************************/
        public PowerAndAngle(double powerParam, double angleParam){
            this.power = powerParam;
            this.angle = angleParam;
        }

        //Getters for power and angle
        public double getPower()  {
            return power;
        }

        public double getAngle()  {
            return angle;
        }
    }

    
    /****************************************************************************************** 
    *
    *    Drive constructor
    * 
    ******************************************************************************************/
    public Drive() {
        //NavX
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

        //PID Controllers
        rotateController = new PIDController(kP, kI, kD);
        rotateController.setTolerance(rotateToleranceDegrees);
        rotateController.enableContinuousInput(-180.0, 180.0);

        autoCrabDriveController = new PIDController(acdP, acdI, acdD);
        autoCrabDriveController.enableContinuousInput(-180.0, 180.0);

        targetController = new PIDController(tP, tI, tD);
        targetController.setTolerance(kLimeLightToleranceDegrees);

        /**
         * LIMELIGHT
         */
        //Network Table
        limelightEntries = NetworkTableInstance.getDefault().getTable("limelight");

        //Network Table Entires
        targetValid = limelightEntries.getEntry("tv");
        targetX     = limelightEntries.getEntry("tx");
        targetY     = limelightEntries.getEntry("ty");
        targetArea  = limelightEntries.getEntry("ta");
        ledMode     = limelightEntries.getEntry("ledMode");
        cameraMode  = limelightEntries.getEntry("camMode");
        stream      = limelightEntries.getEntry("stream");
        pipeline    = limelightEntries.getEntry("pipeline");
        
        /**
		 * Limelight Modes
		 */
		//Force the LED's to on to start the match
		ledMode.setNumber(0);
		//Set limelight mode to vision processor
		cameraMode.setNumber(0);
		//Sets limelight streaming mode to Standard (The primary camera and the secondary camera are displayed side by side)
		stream.setNumber(0);
		//Sets limelight pipeline to 0 (ON_TARMAC)
		pipeline.setNumber(0);
    }


    /****************************************************************************************** 
    *
    *    calcSwerve()
    *    <p> For each wheel, the inputted X, Y, Z, and individual angle for rotation are used to calculate the angle and power 
    * 
    ******************************************************************************************/
    private PowerAndAngle calcSwerve(double crabX, double crabY, double rotatePower, double rotateAngle, boolean fieldDriveEnabled){
        double swerveX;
        double swerveY;
        double swervePower;
        double swerveAngle;
        double rotateX;
        double rotateY;

        //If field drive is active then the crab drive values are shifted based on gyro reading
        if (fieldDriveEnabled) {
            double crabPower = Math.sqrt((crabX * crabX) + (crabY * crabY));
            double crabAngle = Math.toDegrees(Math.atan2(crabX, crabY));
            crabAngle -= ahrs.getYaw();

            crabX = Math.sin(Math.toRadians(crabAngle)) * crabPower;
            crabY = Math.cos(Math.toRadians(crabAngle)) * crabPower;
        }
       
        /**
         * The incomming rotate angle will cause the robot to rotate counter-clockwise
         * the incomming power is negative for a counter-clockwise rotation and vise versa for clockwise
         * therefore, we want power to be positive to achieve a counter-clockwise rotation
         * which means that we have to multiply the power by negative 1  
         */ 
        rotateX = (-1 * rotatePower) * Math.sin(Math.toRadians(rotateAngle));
        rotateY = (-1 * rotatePower) * Math.cos(Math.toRadians(rotateAngle));

        swerveX = crabX + rotateX;
        swerveY = crabY + rotateY;

        swervePower = Math.sqrt((swerveX * swerveX) + (swerveY * swerveY));
        swerveAngle = Math.toDegrees(Math.atan2(swerveX, swerveY));

        //If we are rotating CCW, and we are not crab driving, then the robot will flip the wheel angles and powers
        //This keeps the wheels in the same position when turning both ways, making small rotations easier
        if ((rotatePower < 0) && (crabX == 0 && crabY == 0)) {
            swervePower *= -1;
            swerveAngle += 180;
        }

        PowerAndAngle swerveNums = new PowerAndAngle(swervePower, swerveAngle);

        return swerveNums;
    }


    /****************************************************************************************** 
    *
    *    teleopSwerve()
    *    <p> Takes X, Y, and Z and rotates each wheel to proper angle and sets correct power
    * 
    ******************************************************************************************/
    public void teleopSwerve(double driveX, double driveY, double rotatePower, boolean fieldDriveEnabled) {
        PowerAndAngle coor;

        coor = calcSwerve(driveX, driveY, rotatePower, rotateRightFrontMotorAngle, fieldDriveEnabled);
        frontRightWheel.rotateAndDrive(coor.getAngle(), coor.getPower());

        coor = calcSwerve(driveX, driveY, rotatePower, rotateLeftFrontMotorAngle, fieldDriveEnabled);
        frontLeftWheel.rotateAndDrive(coor.getAngle(), coor.getPower());

        coor = calcSwerve(driveX, driveY, rotatePower, rotateRightRearMotorAngle, fieldDriveEnabled);
        rearRightWheel.rotateAndDrive(coor.getAngle(), coor.getPower());

        coor = calcSwerve(driveX, driveY, rotatePower, rotateLeftRearMotorAngle, fieldDriveEnabled);
        rearLeftWheel.rotateAndDrive(coor.getAngle(), coor.getPower());
    }


    /****************************************************************************************** 
    *
    *    teleopCrabDrive()
    *    <p> Only uses X and Y to crab drive the robot
    * 
    ******************************************************************************************/
    public void teleopCrabDrive(double wheelAngle, double drivePower){
        frontLeftWheel.rotateAndDrive(wheelAngle, drivePower);
        frontRightWheel.rotateAndDrive(wheelAngle, drivePower);
        rearLeftWheel.rotateAndDrive(wheelAngle, drivePower);
        rearRightWheel.rotateAndDrive(wheelAngle, drivePower);
    }

   
    /****************************************************************************************** 
    *
    *    autoCrabDrive()
    *    <p> Drives robot for certain distance at a given heading and speed
    *    <p> Generic function for autoCrabDrive with default power of 0.6
    *    @param distanceInFeet
    *    @param targetHeading
    *    @return Robot Status
    * 
    ******************************************************************************************/
    public int autoCrabDrive(double distance, double targetHeading) { 
        return autoCrabDrive(distance, targetHeading, 0.6);
    }


    /****************************************************************************************** 
    *
    *    autoCrabDrive()
    *    <p> Drives robot for certain distance at a given heading and speed
    *    <p> Distance has to be positive
    *    <p> Initial orientation of robot is maintained throughout function
    *    @param distanceInFeet
    *    @param targetHeading
    *    @param power
    *    @return Robot Status
    * 
    ******************************************************************************************/
    public int autoCrabDrive(double distance, double targetHeading, double power) {

        double encoderCurrent = getAverageEncoder(); //Average of 4 wheels

        //First time through initializes target values
        if (firstTime == true) {
            firstTime = false;
            targetOrientation = ahrs.getYaw();
            encoderTarget = encoderCurrent + (ticksPerFoot * distance);
        }

        //Halves speed within 3 feet of target, if total distance is at least 5 feet
        if ((encoderCurrent + (3 * ticksPerFoot) > encoderTarget) && distance > 5) {
            power = power / 2;
        }

        double orientationError;
        double x = power * Math.sin(Math.toRadians(targetHeading));
        double y = power * Math.cos(Math.toRadians(targetHeading));

        if (distance < 0){
            System.out.println("Error from autoCrabDrive(), negative distance not allowed");
            return Robot.DONE;
        }
        
        //Adjusts wheel angles
        orientationError = autoCrabDriveController.calculate(ahrs.getYaw(), targetOrientation); 
        teleopSwerve(x, y, orientationError, false);

        //Checks if target distance has been reached, then ends function if so
        if (encoderCurrent >= encoderTarget) {
            firstTime = true;
            stopWheels();
            rotateController.reset();
            return Robot.DONE;
        } 
        else {
            return Robot.CONT;
        }

    }


    /****************************************************************************************** 
    *
    *    teleopRotate()
    *    <p> Only uses Z to rotate robot
    *    <p> This function negates rotatePower in order to make positive inputs turn the robot clockwise
    * 
    ******************************************************************************************/
    public void teleopRotate(double rotatePower) {
        frontRightWheel.rotateAndDrive(rotateRightFrontMotorAngle, rotatePower * -1);
        frontLeftWheel.rotateAndDrive(rotateLeftFrontMotorAngle, rotatePower * -1);
        rearRightWheel.rotateAndDrive(rotateRightRearMotorAngle, rotatePower * -1);
        rearLeftWheel.rotateAndDrive(rotateLeftRearMotorAngle, rotatePower * -1);
    }


    /****************************************************************************************** 
    *
    *    autoRotate()
    *    <p> Rotates robot to inputted angle
    * 
    ******************************************************************************************/
    public int autoRotate(double degrees) {
        double rotateError;
        long currentMs = System.currentTimeMillis();

        if (rotateFirstTime == true) {
            rotateFirstTime = false;
            count = 0;
            timeOut = currentMs + 2500; //Makes the time out 2.5 seconds
        }

        if (currentMs > timeOut) {
			count = 0;
            rotateFirstTime = true;
            
			System.out.println("Timed out");
            stopWheels();
            return Robot.FAIL;
		}

		// Rotate
        rotateError = rotateController.calculate(ahrs.getYaw(), degrees);
        rotateError = MathUtil.clamp(rotateError, -0.5, 0.5);
        System.out.println(rotateError + " " + ahrs.getYaw());
		teleopRotate(rotateError);

		// CHECK: Routine Complete
		if (rotateController.atSetpoint() == true) {
            count++;            

			if (count == ON_ANGLE_COUNT) {
				count = 0;
                rotateFirstTime = true;
                rotateController.reset();
                stopWheels();                                
                return Robot.DONE;
            }
            else {
				return Robot.CONT;
			}
		}
		else {    
			count = 0;
            return Robot.CONT;
		}
    }


    /****************************************************************************************** 
    *
    *    circle()
    *    <p> Moves robot around circle with given radius (radius is from center of circle to center of robot)
    * 
    ******************************************************************************************/
    public void circle(double radiusFeet) {
        double radius = radiusFeet*12;

        //Finds angle of the radius to each wheel, used to find the angle the wheels need to go to
        double innerAngle = Math.toDegrees(Math.atan2(robotWidth/2, radius - (robotLength/2)));
        double outerAngle = Math.toDegrees(Math.atan2(robotWidth/2, radius + (robotLength/2)));

        //The distance that each wheel is from the center of the circle is found with the pythagorean theorem
        double innerDist = Math.pow(   Math.pow((robotWidth/2), 2) + Math.pow(radius - robotLength/2, 2),    0.5);
        double outerDist = Math.pow(   Math.pow((robotWidth/2), 2) + Math.pow(radius + robotLength/2, 2),    0.5);

        //The ratio between the inner and outer speeds is equal to the ratio of their distances
        double outerSpeed = 0.25; //Sets basis for speed of turning
        double innerSpeed = outerSpeed * (innerDist/outerDist);

        frontLeftWheel.rotateAndDrive(innerAngle + 90, innerSpeed);
        frontRightWheel.rotateAndDrive(-90 - innerAngle, -1*innerSpeed);

        rearLeftWheel.rotateAndDrive(outerAngle + 90, outerSpeed);
        rearRightWheel.rotateAndDrive(-90 - outerAngle, -1*outerSpeed);
    }


    /****************************************************************************************** 
    *
    *    spiral()
    *    <p> Robot moves forward while spinning around
    * 
    ******************************************************************************************/
    public void spiral() {
        teleopSwerve(0, 0.3, 0.3, true);
    }


    /****************************************************************************************** 
    *
    *    autoAdjustWheels()
    *    <p> Rotates wheels to desired angle (between -180 and 180)
    * 
    ******************************************************************************************/
    public int autoAdjustWheels(double degrees) {
        if (Math.abs(degrees) > 180) {
            System.out.println("ERROR: Drive::autoAdjustWheels() domain exception. Expected domain from -180 to 180, got " + degrees);
            return Robot.FAIL;
        }

        long currentMs = System.currentTimeMillis();

        if (rotateFirstTime == true) {
            rotateFirstTime = false;
            count = 0;
            timeOut = currentMs + 2500; //Originally 500ms
        }

        if (currentMs > timeOut) {
			count = 0;
            rotateFirstTime = true;
            
			System.out.println("Timed out");
            stopWheels();
            return Robot.FAIL;
		}

		// Rotate
        int FR = frontRightWheel.rotateAndDrive(degrees, 0);
        int FL = frontLeftWheel.rotateAndDrive(degrees, 0);
        int BR = rearRightWheel.rotateAndDrive(degrees, 0);
        int BL = rearLeftWheel.rotateAndDrive(degrees, 0);

        //Checks if all wheels are at target angle
        if (FR == Robot.DONE && FL == Robot.DONE && BR == Robot.DONE && BL == Robot.DONE) {
            stopWheels();
            return Robot.DONE;
        }
        else {
            return Robot.CONT;
        }
    }


    /****************************************************************************************** 
    *
    *    getAverageEncoder()
    *    <p> Returns average value of all 4 wheels' encoders
    * 
    ******************************************************************************************/
    private double getAverageEncoder(){
        //Encoder value get statements
        double frontRight = frontRightWheel.getEncoderValue();
        double frontLeft  = frontLeftWheel.getEncoderValue();
        double backRight  = rearRightWheel.getEncoderValue();
        double backLeft   = rearLeftWheel.getEncoderValue();

        //Creates a DoubleStream
        DoubleStream stream = DoubleStream.of(frontRight, frontLeft, backRight, backLeft);

        //Averages the DoubleStream and converts to a double
        double average = stream.average().getAsDouble();
        
        //Returns the average
        return average;
    }


    /****************************************************************************************** 
    *
    *    stopWheels()
    *    <p> Turns off all motors instead of turning wheels back to 0 degrees
    * 
    ******************************************************************************************/
    public void stopWheels(){
        frontLeftWheel.setDriveMotorPower(0);
        frontRightWheel.setDriveMotorPower(0);
        rearLeftWheel.setDriveMotorPower(0);
        rearRightWheel.setDriveMotorPower(0);

        frontLeftWheel.setRotateMotorPower(0);
        frontRightWheel.setRotateMotorPower(0);
        rearLeftWheel.setRotateMotorPower(0);
        rearRightWheel.setRotateMotorPower(0);
    }


    /****************************************************************************************** 
    *
    *    LIMELIGHT METHODS
    * 
    ******************************************************************************************/
    /**
     * Limelight targeting using PID
     * @return program status
     */
	public int limelightPIDTargeting(TargetPipeline pipeline) {
        //Variables
		double m_LimelightCalculatedPower = 0;
        long   currentMs = System.currentTimeMillis(); //Gets the current time

        //Constants
        final int  TIME_OUT_SEC   = 5;
        final long TIME_OUT_MSEC = TIME_OUT_SEC * 1000;

        //Sets the required pipeline
        changePipeline(pipeline);

		if (limeLightFirstTime == true) {
            //Sets limeLightFirstTime to false
            limeLightFirstTime = false;

            //Resets the variables for tracking targets
			noTargetCount    = 0;
            targetLockedCount = 0;
            
            //Sets and displays the forced time out
			timeOut = currentMs + TIME_OUT_MSEC;
            System.out.println("Limelight timeOut " + TIME_OUT_SEC + " seconds");
            
            //Turns the limelight on
            changeledMode(LEDState.ON);
		}

		// Whether the limelight has any valid targets (0 or 1)
        double tv = get_tv();
        //System.out.println("tv: " + tv);
		// Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) [54 degree tolerance]
		double tx = get_tx();
        //System.out.println("tx: " + tx);

		// Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) [41 degree tolerance]
        //double ty = get_ty();
        //System.out.println("ty: " + ty);
		// Target Area (0% of image to 100% of image) [Basic way to determine distance]
        //double ta = get_ta();
        //System.out.println("ta: " + ta);

		if (tv < 1.0) {
            stopWheels();

            //Adds one to the noTargetCount (will exit this program if that count exceedes 5) 
			noTargetCount++;

			if (noTargetCount <= FAIL_DELAY) {
                //Tells the robot to continue searching
				return Robot.CONT;
			}
			else {
                //Reset variables
				noTargetCount      = 0;
                targetLockedCount  = 0;
                limeLightFirstTime = true;
                targetController.reset();

                //Stops the robot
                stopWheels();
                
                //Returns the error code for failure
				return Robot.DONE;
			}
		}
        else {
            //Keeps noTargetCount at 0
            noTargetCount = 0;
		}

        // Rotate
		m_LimelightCalculatedPower = targetController.calculate(tx, 0.0);
        m_LimelightCalculatedPower = MathUtil.clamp(m_LimelightCalculatedPower, -0.50, 0.50);
		teleopRotate(m_LimelightCalculatedPower * -1);
		//System.out.println("Pid out: " + m_LimelightCalculatedPower);

		// CHECK: Routine Complete
		if (targetController.atSetpoint() == true) {
            targetLockedCount++;
            
			//System.out.println("On target");
		}

		if (targetLockedCount >= ON_TARGET_COUNT) {
            //Reset variables
            targetLockedCount = 0;
            noTargetCount     = 0;
            limeLightFirstTime = true;
            targetController.reset();
            
            //Stops the robot
			stopWheels();

            //Returns the error code for success
			return Robot.DONE;
        }
        
		// limelight time out readjust
		if (currentMs > timeOut) {
            //Resets the variables
            targetLockedCount = 0;
            noTargetCount     = 0;
            limeLightFirstTime = true;
            targetController.reset();
            
            //Stops the robot
            stopWheels();
            
            //Prints the timeout
            System.out.println("timeout " + tx + " Target Acquired " + tv);

            //Turns LED's off
            changeledMode(LEDState.OFF);

            //Returns the error code for failure
			return Robot.FAIL;
        }
        
		return Robot.CONT;   
    }

    /**
     * Gets the value of tv
     * @return validTarget
     */
    public double get_tv() {
        return targetValid.getDouble(0.00);
    }

    /**
     * Gets the value of tx
     * @return xOffset
     */
    public double get_tx() {
        return targetX.getDouble(0.00);
    }

    /**
     * Gets the value of ty
     * @return YOffset
     */
    public double get_ty() {
        return targetY.getDouble(0.00);
    }

    /**
     * Gets the value of ta
     * @return area
     */
    public double get_ta() {
        return targetArea.getDouble(0.00);
    }

    /**
     * Chanegs the current pipeline on the Limelight
     * @param pipelineName
     */
    public void changePipeline(TargetPipeline pipelineName) {
        //Makes it a bit easier to change pipelines
        if (pipelineName == TargetPipeline.ON_TARMAC) {
            // Configures the limelight for on tarmac targeting
            pipeline.setNumber(0);
        }
        else if (pipelineName == TargetPipeline.OFF_TARMAC) {
            // Configures the limelight for on tarmac targeting
            pipeline.setNumber(1);
        }
        else {
            // Configures the limelight to do nothing
            pipeline.setNumber(2);
        }
    }

    /**
     * Changes the limelight LED mode
     * @param state
     */
    public void changeledMode(LEDState mode) {
        //Makes it easier to change the LED mode
        if (mode == LEDState.ON) {
            // Sets limelight to on
            ledMode.setNumber(0);
        }
        else if (mode == LEDState.OFF) {
            // Sets limelight to on
            ledMode.setNumber(1);
        }
        else {
            // Sets limelight to on
            ledMode.setNumber(0);
        }
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    public void testAverageLocation() {
        double leftX = (frontLeftWheel.getXPosition() + rearLeftWheel.getXPosition())/2;
        double leftY = (frontLeftWheel.getYPosition() + rearLeftWheel.getYPosition())/2;
        double rightX = (frontRightWheel.getXPosition() + rearRightWheel.getXPosition())/2;
        double rightY = (frontRightWheel.getYPosition() + rearRightWheel.getYPosition())/2;

        double rotationAmount = leftY - rightY;
        double averageX = (leftX + rightX)/2;
        double averageY = (leftY + rightY)/2;

        SmartDashboard.putNumber("ahrs x", ahrs.getDisplacementX());
        SmartDashboard.putNumber("ahrs y", ahrs.getDisplacementY());
        SmartDashboard.putNumber("ahrs z", ahrs.getDisplacementZ());

        SmartDashboard.putNumber("Average X", averageX);
        SmartDashboard.putNumber("Average Y", averageY);
        SmartDashboard.putNumber("Est Rotation", rotationAmount);
    }

    public double testAverageX() {
        double leftX = (frontLeftWheel.getXPosition() + rearLeftWheel.getXPosition())/2;
        double rightX = (frontRightWheel.getXPosition() + rearRightWheel.getXPosition())/2;

        return (leftX + rightX)/2;
    }

    public double testAverageY() {
        double leftY = (frontLeftWheel.getYPosition() + rearLeftWheel.getYPosition())/2;
        double rightY = (frontRightWheel.getYPosition() + rearRightWheel.getYPosition())/2;

        return (leftY + rightY)/2;
    }

    public int goToPoint(double x, double y, double angle) {
        double xPower = (x - testAverageX())/10;
        double yPower = (y - testAverageY())/10;
        double rotationPower = (angle - ahrs.getYaw())/200;

        if (Math.abs(xPower) < 0.04 && Math.abs(yPower) < 0.04 && Math.abs(rotationPower) < 0.04) {
            //resetKinematics();
            return Robot.DONE;
        }

        //Clamps max acceleration
        // xPower = MathUtil.clamp(xPower, previousXPower - 0.01, previousXPower + 0.01);
        // yPower = MathUtil.clamp(yPower, previousYPower - 0.01, previousYPower + 0.01);
        // rotationPower = MathUtil.clamp(rotationPower, previousRotatePower - 0.01, previousRotatePower + 0.01);
        // System.out.println("Acc. clamped y power: " + yPower);

        //Clamps max power
        xPower = MathUtil.clamp(xPower, -0.35, 0.35);
        yPower = MathUtil.clamp(yPower, -0.35, 0.35);
        rotationPower = MathUtil.clamp(rotationPower, -0.2, 0.2);

        previousXPower = xPower;
        previousYPower = yPower;
        previousRotatePower = rotationPower;

        teleopSwerve(xPower, yPower, rotationPower, true);

        return Robot.CONT;
    }

    public void resetKinematics() {
        frontLeftWheel.setXPosition(0);
        frontLeftWheel.setYPosition(0);
        frontRightWheel.setXPosition(0);
        frontRightWheel.setYPosition(0);
        rearLeftWheel.setXPosition(0);
        rearLeftWheel.setYPosition(0);
        rearRightWheel.setXPosition(0);
        rearRightWheel.setYPosition(0);
    }
    public void testWheel(){
        rearRightWheel.setDriveMotorPower(-0.5);
    }
    
    public void testRotate(){
        double power = -.2;
        frontLeftWheel.setRotateMotorPower(power);
        frontRightWheel.setRotateMotorPower(power);
        rearLeftWheel.setRotateMotorPower(power);
        rearRightWheel.setRotateMotorPower(power);
        System.out.println("Degrees: " + rearLeftWheel.getRotateMotorPosition());
    }

    public void testPID() {
        frontLeftWheel.rotateAndDrive(0, 0);
    }

    public void testEncoder(){
        System.out.println("FR encoder: " + frontRightWheel.getEncoderValue());
    }

    public void testWheelAngle(){
        //Use this to calibrate wheel angle sensors
        //Offset in wheel constructor should be the returned value * -1
        System.out.println("FL Angle: " + frontLeftWheel.testWheelAngle());
        System.out.println("FR Angle: " + frontRightWheel.testWheelAngle());
        System.out.println("RL Angle: " + rearLeftWheel.testWheelAngle());
        System.out.println("RR Angle: " + rearRightWheel.testWheelAngle());
    }

    public void testLimelightTargeting() {
        System.out.print("tv: " + get_tv() + "      |     ");
        System.out.println("tx: " + get_tx());
    }

}

// End of the Drive Class