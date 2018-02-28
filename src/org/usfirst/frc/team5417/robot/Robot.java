
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5417.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import org.usfirst.frc.team5417.robot.XBoxController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot implements PIDSource, PIDOutput {

	//Initialize Auto modes
	DriverStation driverstation = DriverStation.getInstance();
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	String autoSelected = "";
	final String defaultAuto = "Default Autonomous";
	final String leftAuto = "Left auto";
	final String centerAuto = "Center auto";
	final String driveStraight = "Drive straight";
	final String rightAuto = "Right auto";

	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	//Motor Controllers
	WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(1);
	WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(2);
	WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(5);
	WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(6);
	VictorSP armMotor1 = new VictorSP(0); 
	//WPI_TalonSRX armMotor1 = new WPI_TalonSRX(3);
	VictorSP armMotor2 = new VictorSP(1); 
	//WPI_TalonSRX armMotor2 = new WPI_TalonSRX(7);
	//WPI_TalonSRX climberMotor1 = new WPI_TalonSRX(0);
	WPI_TalonSRX climberMotor1 = new WPI_TalonSRX(4);
	
	//Using VictorSP's for the intake motors
	VictorSP leftIntake = new VictorSP(3); // change to 0 and 1 at competition
	VictorSP rightIntake = new VictorSP(4);
	
	//Initialize compressor which is used for changing gears, brakes on the climber, and climbing pistons 
	Compressor compressor = new Compressor();
	
	//Initialize Solenoids for pistons
	Solenoid gearShift = new Solenoid(0);
	Solenoid climbPistons = new Solenoid(3);
	Solenoid wristPistons = new Solenoid(2);
	Solenoid bigPistons = new Solenoid(1);
	
	//Initialize limit switches for Arm
	DigitalInput limitSwitchTop = new DigitalInput(4);
	DigitalInput limitSwitchBottom = new DigitalInput(5);
	DigitalInput encoderLeftA = new DigitalInput(0);
	DigitalInput encoderLeftB = new DigitalInput(1);
	DigitalInput encoderRightA = new DigitalInput(2);
	DigitalInput encoderRightB = new DigitalInput(3);
	
	//Initialize servo for the ramp/lift
	Servo rampServo = new Servo(2);
	
	Encoder leftEncoder = new Encoder(encoderLeftA, encoderLeftB, true);
	Encoder rightEncoder = new Encoder(encoderRightA, encoderRightB);
	
	//Initialize PID for drive system and for the 4bar arm system
	PIDController navxPID = new PIDController(.01, 0, .1, this, this); /// changed PID values
	ArmPID leftArmPIDMoving = new ArmPID(leftEncoder, PIDSourceType.kRate, armMotor1, .002, 0.0001, 0);
	ArmPID rightArmPIDMoving = new ArmPID(rightEncoder, PIDSourceType.kRate, armMotor2, .002, 0.00005, .0001);
	
	//Timer for stuff
	Timer timer = new Timer();

	//Initializing Drive Train
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
	
	//Initializing the controllers
	XBoxController driverStick = new XBoxController(new Joystick(0));
	XBoxController manipulatorStick = new XBoxController(new Joystick(1));

	//Initializing Drivetrain Encoders
	int leftPosition = 0;
	int rightPosition = 0;
	boolean buttonstatus = false;
	//NavxPID navxPID = new NavxPID(.1,20000, 0);
	double left;
	double right;
	AHRS navx;
	ContinuousAngleTracker dsCAT;
	ContinuousAngleTracker tlCAT;
	ContinuousAngleTracker trCAT;
	double yaw = 0;
	double multiplier = 0;
	boolean pressed = false;
	// Trajectory.Config config = new
	// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
	// Trajectory.Config.SAMPLES_FAST, .05, .1, .5, .1);
	// Waypoint[] points = new Waypoint[] {
	// new Waypoint(0,0, Pathfinder.d2r(90)),
	// new Waypoint(0,3,Pathfinder.d2r(90)),
	// //new Waypoint(2,7,Pathfinder.d2r(45))
	// };
	// Trajectory trajectory = Pathfinder.generate(points, config);
	// TankModifier modifier = new TankModifier(trajectory).modify(.66);
	// Trajectory leftTrajectory = modifier.getLeftTrajectory();
	// Trajectory rightTrajectory = modifier.getRightTrajectory();
	// EncoderFollower leftFollower = new EncoderFollower(leftTrajectory);
	// EncoderFollower rightFollower = new EncoderFollower(rightTrajectory);
	double leftSpeed = 0;
	double rightSpeed = 0;
	int initLeftPosition;
	int initRightPosition;
	double correction = 0;
	double initialYaw;
	double currentYaw;
	double targetYaw;
	boolean run = false;
	double lastYaw;
	boolean turnCompleted = false;
	double movingSetpoint = 0;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		navx = new AHRS(SerialPort.Port.kUSB);
		navx.reset();
		
		leftEncoder.setDistancePerPulse(0.17578125);
		rightEncoder.setDistancePerPulse(0.17578125);
		

		dsCAT = new ContinuousAngleTracker();
		tlCAT = new ContinuousAngleTracker();
		trCAT = new ContinuousAngleTracker();
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		rightMotor1.setInverted(true);
		rightMotor2.setInverted(true);
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		CameraServer server = CameraServer.getInstance();
		server.startAutomaticCapture("cam0", 0);
		gearShift.set(false);
		climbPistons.set(false);
		wristPistons.set(false);
		rampServo.set(.5);
		leftArmPIDMoving.setOutputRange(-1, 1);
		leftArmPIDMoving.setAbsoluteTolerance(5);
		rightArmPIDMoving.setOutputRange(-1, 1);
		rightArmPIDMoving.setAbsoluteTolerance(5);
		
		resetNavxPID();
		m_autoSelected = kCustomAuto;
		// leftFollower.configurePIDVA(.8, 0, 0, 127);
		

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector",
		// kDefaultAuto);
		if (SmartDashboard.getBoolean("DB/Button 0", false)) {
			autoSelected = "Drive Straight";
		}
		else if (SmartDashboard.getBoolean("DB/Button 1", false)) {
			autoSelected = "Left auto";
		}
		else if (SmartDashboard.getBoolean("DB/Button 2", false)) {
			autoSelected = "Center auto";
		}
		else if (SmartDashboard.getBoolean("DB/Button 3", false)) {
			autoSelected = "Right auto";
		}
		
		SmartDashboard.putString("DB/String 1","auto:" + autoSelected);
		
		gearShift.set(false);
		climbPistons.set(false);
		wristPistons.set(false);
		initLeftPosition = leftMotor1.getSelectedSensorPosition(0);
		initRightPosition = -rightMotor1.getSelectedSensorPosition(0);
		getSwitchString(); //if true then is left
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// update the continuous angle tracker with the current angle so it can track
		// angles over time
		dsCAT.nextAngle(navx.getYaw());
		switch (autoSelected) {
		case kCustomAuto:
			break;
		case kDefaultAuto:
			//
			break;
		case leftAuto:		//left side autonomous
			//if switch is left 
			if (getSwitchString()) {
				 driveDistance(149);
				 turnRight();
				 Timer timer = new Timer();
				 timer.start();
				 while (timer.get()<1) {
				 leftIntake.set(-1);
				 rightIntake.set(1);
				 }
				 leftIntake.set(0);
				 rightIntake.set(0);
				 left = 0;
				 right = 0;			 
				 break;
			}
			//if switch is right
			if (!getSwitchString()) {
				 //drive behind switch and do the thing
				SmartDashboard.putString("DB/String 3", "is right side switch");
				break;
			}
			break;
		case centerAuto:
			driveDistance(125); // just drive straight for now
			SmartDashboard.putString("DB/String 3", "is driveStaight");
			break;
		case rightAuto:			//for right side autonomous
			//if switch is right
			if (!getSwitchString()) {
				 driveDistance(149);
				 turnLeft();
				 Timer timer = new Timer();
				 timer.start();
				 while (timer.get()<1) {
				 leftIntake.set(-1);
				 rightIntake.set(1);
				 }
				 leftIntake.set(0);
				 rightIntake.set(0);
				 left = 0;
				 right = 0;
				 break;
			}
			//if switch is left
			if (getSwitchString()) {
				 //drive behind switch and do the thing
				SmartDashboard.putString("DB/String 3", "is left side switch");
				break;
			}
			break;
		default:
			break;
		}
	}

	@Override
	public void teleopInit() {
		compressor.start();
		timer.start();
		run = false;
		leftArmPIDMoving.enable();
		rightArmPIDMoving.enable();
		// Arm.setSetpoint(motor.getSelectedSensorPosition(0));
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		// update the continuous angle tracker with the current angle so it can track
		// angles over time
		dsCAT.nextAngle(navx.getYaw());

		SmartDashboard.putNumber("left", left);
		SmartDashboard.putNumber("right", right);

		rightPosition = -rightMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putString("DB/String 2", "Right Position: " + rightPosition);
		leftPosition = leftMotor1.getSelectedSensorPosition(0);
		 SmartDashboard.putString("DB/String 0", "Left Position: " + leftPosition);
		// float yaw = navx.getYaw();
		yaw = dsCAT.getAngle();
		SmartDashboard.putString("DB/String 4", "Z: " + (float) yaw);

//		Gear Shift
		if (driverStick.isFirstLBPressed()) {
			gearShift.set(false);
		} else if (driverStick.isFirstRBPressed()) {
			gearShift.set(true);
		}
		
//		Drive Straight Initialization
		if (driverStick.isFirstYPressed()) {
			// double currentYaw = navx.getYaw();

			dsCAT.reset();

			float currentNavxYaw = navx.getYaw();
			dsCAT.setAngleAdjustment(-currentNavxYaw);
			dsCAT.nextAngle(currentNavxYaw);

			double currentYaw = dsCAT.getAngle();
			resetNavxPID();
			navxPID.setSetpoint(currentYaw);
			SmartDashboard.putString("DB/String 5", "sp: " + (float) currentYaw);

			timer.reset();
		}// else if (driverStick.isFirstXPressed()) {
			/*
			 * initialYaw = navx.getYaw(); currentYaw = initialYaw; lastYaw = currentYaw;
			 * targetYaw = initialYaw + 75; if (targetYaw > 180) { targetYaw = (targetYaw -
			 * 360); } turnCompleted = false;
			 */
			// turnLeft();
	//	}	
//		SmartDashboard.putString("DB/String 3", "upperlimit: " + limitSwitchTop.get());
//		SmartDashboard.putString("DB/String 4", "lowerlimit: " + limitSwitchBottom.get());
		
	//Arm Motors		
		if (manipulatorStick.getRTValue() > .1) {
			armMotor1.set(manipulatorStick.getRTValue());
			armMotor2.set(-manipulatorStick.getRTValue());
			
		} 
		else if (manipulatorStick.getLTValue() > .1) {
			armMotor1.set(-manipulatorStick.getLTValue());
			armMotor2.set(manipulatorStick.getLTValue());
			
		} else {
			armMotor1.set(0);
			armMotor2.set(0);
			
		}
		
		
		
		
		
		
		/*	if (manipulatorStick.getRTValue() > .2 && limitSwitchTop.get()) {
			movingSetpoint = manipulatorStick.getRTValue() * 40;
			leftArmPIDMoving.setSetpoint(movingSetpoint);
			rightArmPIDMoving.setSetpoint(movingSetpoint);
			pressed = true;
		} else if (manipulatorStick.getLTValue() > .2 && limitSwitchBottom.get()) {
			movingSetpoint = manipulatorStick.getLTValue() * -40;
			leftArmPIDMoving.setSetpoint(movingSetpoint);
			rightArmPIDMoving.setSetpoint(movingSetpoint);
			pressed = true;
		} else {
			if (pressed) {
				movingSetpoint = 0;
				leftArmPIDMoving.setSetpoint(movingSetpoint);
				rightArmPIDMoving.setSetpoint(movingSetpoint);
				pressed = false;
			}
		}*/
		
//		Climber goes up
//		if (manipulatorStick.isStartHeldDown()) {
//			climberMotor1.set(1);
//			armMotor1.set(.4);
//		}
//		else {
//			climberMotor1.set(0);
//			armMotor1.set(0);
//		}
		
//		Climber goes down
		if (manipulatorStick.isBackHeldDown()) {
			climberMotor1.set(-1);
			//armMotor1.set(-.4);
		}
		else {
			climberMotor1.set(0);
			//armMotor1.set(0);
		}
		
		SmartDashboard.putString("DB/String 9", "Right motor: " + armMotor2.get());
		SmartDashboard.putString("DB/String 8", "Left Motor: " + armMotor1.get());

//		Elevator Brake Pistons
		if (manipulatorStick.isFirstRBPressed()) {
			climbPistons.set(true);
		} else if (manipulatorStick.isFirstLBPressed()) {
			climbPistons.set(false);
		}
		
//		Wrist Piston
		if (manipulatorStick.isFirstYPressed()) {
			wristPistons.set(true);
			} else if (manipulatorStick.isFirstAPressed()){
			wristPistons.set(false);
		}
		
//		Big Lift Pistons		
		if (manipulatorStick.isFirstBPressed()) {
			bigPistons.set(true);
		}
		else if (manipulatorStick.isFirstXPressed()) {
			bigPistons.set(false);
		}

//		Manipulator Intake		
		if (Math.abs(manipulatorStick.getLYValue()) > .2) {
			leftIntake.set(manipulatorStick.getLYValue());
		} else
			leftIntake.set(0);
		
		if (Math.abs(manipulatorStick.getRYValue()) > .2) {
			rightIntake.set(-manipulatorStick.getRYValue());
		} else
			rightIntake.set(0);
//		Servo for ramp
		if (manipulatorStick.isFirstDpadleftPressed()) {
			rampServo.set(.2);
		}
		else if (manipulatorStick.isFirstDpadrightPressed()) {
			rampServo.set(.6);
		}
//		Drive Straight		
		if (driverStick.isYHeldDown()) {
			SmartDashboard.putNumber("c", correction);
			double speedMultiplier = getMultiplier(timer.get(), 1.5);
			left = right = .8 * speedMultiplier;
			left = left + correction; //// IF MOTORS HAVE BEEN FLIPPED, FLIP SIGNS
			right = right - correction; // what that ^ says
			SmartDashboard.putString("DB/String 3", "corr " + (float) correction);
			m_drive.tankDrive(left, right);
		}
		// else if (driverStick.isBHeldDown()) {
		// double speedMultiplier = getSlowDownMultiplier(timer.get(), 1.5);
		// left = right = .8 * speedMultiplier;
		// left = left - correction;
		// right = right + correction;
		// m_drive.tankDrive(left, right);
		// SmartDashboard.putNumber("l", (leftSpeed - correction));
		// SmartDashboard.putNumber("r", (rightSpeed + correction));
		// } else if (driverStick.isXHeldDown()) {
		//
		//// if (currentYaw < targetYaw && !turnCompleted) {
		//// m_drive.tankDrive(-.7, .7);
		//// currentYaw = navx.getYaw();
		//// } else if (initialYaw > 90 && currentYaw > 90 && !turnCompleted) {
		//// m_drive.tankDrive(-.7, .7);
		//// currentYaw = navx.getYaw();
		//// } else if (lastYaw < targetYaw && currentYaw > targetYaw && !turnCompleted)
		// {
		//// turnCompleted = true;
		//// m_drive.tankDrive(.3, -.3);
		//// Timer.delay(.2);
		//// }
		//
		//// else {
		//// m_drive.tankDrive(0, 0);
		//// }
		// lastYaw = currentYaw;
		// }
		else {
			left = -driverStick.getLYValue();
			right = -driverStick.getRYValue();
			m_drive.tankDrive(left, right);
		}
		SmartDashboard.putNumber("left arm encoder", leftEncoder.pidGet());
		SmartDashboard.putNumber("right arm encoder", rightEncoder.pidGet());
		SmartDashboard.putBoolean("top switch", limitSwitchTop.get());
		SmartDashboard.putBoolean("bottom switch", limitSwitchBottom.get());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	@Override
	public void pidWrite(double output) {
		SmartDashboard.putString("DB/String 6", "PID Corr: " + output);
		correction = output;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {

		// return navx.getYaw();
		return dsCAT.getAngle();
	}

	public int inchesToTicks(double inches) {
		int ticks = (int) (inches * 4096 / 15.70794);
		return ticks;
	}

	public double getMultiplier(double time, double maxTime) {
		double multiplier = (Math.pow(time + .5, 2)) / (Math.pow(maxTime, 2));
		if (multiplier < 0) {
			return 0;
		} else if (multiplier > 1) {
			return 1;
		} else {
			return multiplier;
		}
	}

	public double getSlowDownMultiplier(double time, double maxTime) {
		double multiplier = (Math.pow(time - maxTime, 2)) / (Math.pow(maxTime, 2));
		if (multiplier < 0 | time > maxTime) {
			return 0;
		} else if (multiplier > 1) {
			return 1;
		} else {
			return multiplier;
		}
	}

	//////////////////////////////// DRIVE "STRAIGHT"
	public void driveDistance(int distance) {
		int ticks = inchesToTicks(distance);
		int count = leftMotor1.getSelectedSensorPosition(0);
		int target = count + ticks;
		double speedMultiplier = 0;
		timer.start();
		multiplier = 0;
		navxPID.setSetpoint(navx.getYaw());
		while (speedMultiplier < 1) {
			speedMultiplier = getMultiplier(timer.get(), 1);
			left = right = .8 * speedMultiplier;
			left = left - correction;
			right = right + correction;
			m_drive.tankDrive(left, right);
			// SmartDashboard.putString("DB/String 0", "acceleratin");
			// SmartDashboard.putString("DB/String 1", "" + speedMultiplier);
		}
		while ((target - leftMotor1.getSelectedSensorPosition(0)) > 7000) {
			m_drive.tankDrive(.8 - correction, .8 + correction);
			// SmartDashboard.putString("DB/String 1", "" + (target -
			// leftMotor1.getSelectedSensorPosition(0)));
		}
		leftPosition = leftMotor1.getSelectedSensorPosition(0);
		rightPosition = -rightMotor1.getSelectedSensorPosition(0);
		timer.reset();
		while (timer.get() < .1) {
			m_drive.tankDrive(-.4, -.4);
		}
		m_drive.tankDrive(0, 0);
		// while (speedMultiplier > 0) {
		// speedMultiplier = getSlowDownMultiplier(timer.get(), 2);
		// left = right = .8 * speedMultiplier;
		// left = left - correction;
		// right = right + correction;
		// m_drive.tankDrive(left, right);
		// SmartDashboard.putString("DB/String 0", "deceleratin");
		// SmartDashboard.putString("DB/String 1", "" + speedMultiplier);
		// }
		// SmartDashboard.putString("DB/String 0", "Auton finished");

		SmartDashboard.putString("DB/String 2", "" + (leftMotor1.getSelectedSensorPosition(0) - leftPosition));
	}

	///////////////// Reset PID
	public void resetNavxPID() {
		navxPID.reset();
		navxPID.setOutputRange(-0.5, .5);
		navxPID.setAbsoluteTolerance(.01);
		navxPID.setInputRange(-180, 180);
		navxPID.enable();
	}

	////////////////////////////////////////// TURN RIGHT
	////////////////////////////////////////// /////////////////////////////////////////////////

	public void turnRight() {
		tlCAT.reset();
		tlCAT.nextAngle(navx.getYaw());
		initialYaw = tlCAT.getAngle();
		SmartDashboard.putString("DB/String 5", "Turning finished: false");
		SmartDashboard.putString("DB/String 7", "initial yaw: " + initialYaw);
		currentYaw = initialYaw;
		targetYaw = initialYaw + 90;
		turnCompleted = false;

		while (!turnCompleted) {
			tlCAT.nextAngle(navx.getYaw());
			currentYaw = tlCAT.getAngle();
			SmartDashboard.putString("DB/String 8", "current yaw: " + currentYaw);
			SmartDashboard.putString("DB/String 5", "turning");
			if (currentYaw < targetYaw - 30) {
				m_drive.tankDrive(.7, -.7);
			} else {
				turnCompleted = true;
				m_drive.tankDrive(-.3, .3);
			} 
		}
		Timer.delay(.7);
		tlCAT.nextAngle(navx.getYaw());
		currentYaw = tlCAT.getAngle();
		while (currentYaw > (2 + targetYaw)) {
			m_drive.tankDrive(-.5, .5);
			SmartDashboard.putString("DB/String 5", "correcting");
			tlCAT.nextAngle(navx.getYaw());
			currentYaw = tlCAT.getAngle();
		}
		
		while (currentYaw < (targetYaw - 2)) {
			m_drive.tankDrive(.5, -.5);
			SmartDashboard.putString("DB/String 5", "correcting");
			tlCAT.nextAngle(navx.getYaw());
			currentYaw = tlCAT.getAngle();
		}
		
		m_drive.tankDrive(0, 0);
		SmartDashboard.putString("DB/String 5", "Turning finished: true");
	}
	
	///////////////////////////// TURN LEFT ////////////////////////////////////////////
	public void turnLeft() {
		tlCAT.reset();
		tlCAT.nextAngle(navx.getYaw());
		initialYaw = tlCAT.getAngle();
		SmartDashboard.putString("DB/String 5", "Turning finished: false");
		SmartDashboard.putString("DB/String 7", "initial yaw: " + initialYaw);
		currentYaw = initialYaw;
		targetYaw = initialYaw - 90;
		turnCompleted = false;

		while (!turnCompleted) {
			tlCAT.nextAngle(navx.getYaw());
			currentYaw = tlCAT.getAngle();
			SmartDashboard.putString("DB/String 8", "current yaw: " + currentYaw);
			SmartDashboard.putString("DB/String 5", "turning");
			if (currentYaw > targetYaw + 30) {
				m_drive.tankDrive(-.7, .7);
			} else {
				turnCompleted = true;
				m_drive.tankDrive(.3, -.3);
			} 
		}
		Timer.delay(.7);
		tlCAT.nextAngle(navx.getYaw());
		currentYaw = tlCAT.getAngle();
		while (currentYaw < (targetYaw - 2)) {
			m_drive.tankDrive(.5, -.5);
			SmartDashboard.putString("DB/String 5", "correcting");
			tlCAT.nextAngle(navx.getYaw());
			currentYaw = tlCAT.getAngle();
		}
		while (currentYaw > (2 + targetYaw)) {
			m_drive.tankDrive(-.5, .5);
			SmartDashboard.putString("DB/String 5", "correcting");
			tlCAT.nextAngle(navx.getYaw());
			currentYaw = tlCAT.getAngle();
		}
		m_drive.tankDrive(0, 0);
		SmartDashboard.putString("DB/String 5", "Turning finished: true");
	}
	
	
	///////////////////////// getting string for autonomous //////////////////////////////
	// if the switch is Left then the return will be true
	// if the switch is Right then the return will be false
	public boolean getSwitchString() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		char switchStr = gameData.charAt(0);
		boolean switchLeft;
		if(switchStr == 'L') {
			switchLeft = true; 
			return switchLeft;
		}
		if(switchStr == 'R') {
			switchLeft = false;
			return switchLeft;
		}
		else return false;
	}
		
}