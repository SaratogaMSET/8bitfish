
package org.usfirst.frc.team649.robot;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.autonomous.autoMaster;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.CommandGroups.FlipRaisedArm;
import org.usfirst.frc.team649.robot.CommandGroups.IntakeWithWheelsAndClose;
import org.usfirst.frc.team649.robot.CommandGroups.RightScaleClose;
import org.usfirst.frc.team649.robot.commands.AngleTalonPID;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.DistanceTalonPID;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.GyroStraightPID;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetCompressorCommand;
import org.usfirst.frc.team649.robot.commands.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.SimpleAuto;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.robot.util.Lidar;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.hal.PDPJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends TimedRobot {

	public static OI oi;
	public static boolean isPIDActive;
	public static boolean isAutoShift;
	public static boolean isVPid;
	public static boolean isHigh;
	public static boolean isTuningPID = false;
	public static double k_p;
	public static double k_i;
	public static double k_d;
	public static int tuningConstant;
	public static DrivetrainSubsystem drive;
	public static GyroSubsystem gyro;
	public Lidar lidar;
	public static ArmSubsystem arm;
	public static LiftSubsystem lift;
	public static Compressor compressor;
	public static autoMaster automaster;
	public static boolean drivePIDRunning;
	public static double lidarValue;
	public Logger logger;
	public FileHandler local;
	public FileHandler usb;
	public File logLocation;
	public String logLocationString;
	public Timer matchTimer;
	public Timer intakeTimer;
	public double teleOpTime = 135;
	public double autoTime = 15;
	public double currentTime;
	public String conversionTime;
	public double modeTime;
	public double distance;
	public static IntakeSubsystem intake;
	public Timer accelTimer;
	public double lidarCount;
	public double prevWinchVel;
	public double accel;
	public double lastAccel;
	public double prevLiftVel;
	public Timer time;
	public Timer timeAccel;

	public double armVelMax;
	public double secondStageLiftMaxVel;
	public double carriageStageMaxVel;
	public double driveVel;
	public double prevDriveVel;
	public double driveAccel;
	public double rightDTMaxVel;
	public double leftDTMaxVel;
	public double maxAccelDrive;
	public static int timeoutMs = 20;
	public double maxLiftVel;
	public double lidarOffset;

	public static boolean isArmPidRunning;
	public static boolean isLiftPidRunning;
	public static boolean isLiftStall;
	public static int liftState;
	public static int liftHalState;
	public static int customLiftPos;
	int timesCalled;
	public static int armState;
	public static int customArmPos;
	public static boolean armIsFront;

	public static EncoderFollower left;
	public static EncoderFollower right;

	public static Trajectory.Config configRightScaleSingle;
	public static Trajectory trajectoryRightScaleSingle;
	public static TankModifier modifierRightScaleSingle;

	public static Trajectory.Config configLeftScaleSingle;
	public static Trajectory trajectoryLeftScaleSingle;
	public static TankModifier modifierLeftScaleSingle;

	public static Trajectory.Config configMiddleRightSingle;
	public static Trajectory trajectoryMiddleRightSingle;
	public static TankModifier modifierMiddleRightSingle;

	public static Trajectory.Config configMiddleLeftSingle;
	public static Trajectory trajectoryMiddleLeftSingle;
	public static TankModifier modifierMiddleLeftSingle;
	// prev state variables leave at bottom

	// these two are for buttons not the actual
	public static boolean autoShiftButtonPrevState;
	public static boolean VPidButtonPrevState;

	public static boolean liftManualPrevState;
	public static boolean armManualPrevState;
	public static boolean prevStateFlipArm;

	public static boolean useLogging;
	public static boolean enteredManualMode;

	@Override
	public void robotInit() {
		useLogging = true;
		drivePIDRunning = false;
		prevWinchVel = 0;
		oi = new OI();
		lidarCount = 0;
		drive = new DrivetrainSubsystem();
		gyro = new GyroSubsystem();
		arm = new ArmSubsystem();
		intake = new IntakeSubsystem();
		intakeTimer = new Timer();
		lift = new LiftSubsystem();
		prevLiftVel = 0;
		maxLiftVel = 0;
		// automaster = new autoMaster();
		lidar = new Lidar(I2C.Port.kOnboard, 0xC4 >> 1);
		compressor = new Compressor(4);
		isPIDActive = false;
		accelTimer = new Timer();
		k_p = drive.getPIDController().getP();
		k_i = drive.getPIDController().getI();
		k_d = drive.getPIDController().getD();
		secondStageLiftMaxVel = 0;
		carriageStageMaxVel = 0;
		distance = 50;
		tuningConstant = 1;
		accel = 0;
		lastAccel = 0;
		time = new Timer();
		timeAccel = new Timer();
		driveAccel = 0;
		driveVel = 0;
		prevDriveVel = 0;
		liftState = 2;
		ManualMode = true;
		armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
		rightDTMaxVel = 0;
		leftDTMaxVel = 0;
		timesCalled = 0;
		lidarOffset = 0;
		lidarValue = lidar.getSample();
		logger = Logger.getLogger("robotLog");
		matchTimer = new Timer();
		logLocationString = "/media/sda1/" + DriverStation.getInstance().getEventName().trim() + "/match number "
				+ DriverStation.getInstance().getMatchNumber() + ".txt";
		logLocation = new File(logLocationString);

		try {
			usb = new FileHandler(logLocationString);
			logger.addHandler(usb);
			SimpleFormatter usbFormatter = new SimpleFormatter();
			usb.setFormatter(usbFormatter);
		} catch (SecurityException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		// Waypoint[] pointsRightScaleSingle = new Waypoint[] {
		// new Waypoint(-12.6,-2.9,0),
		// new Waypoint(-4.3,-2.9,Pathfinder.d2r(45)),
		// new Waypoint(-2.05,0,0),
		// new Waypoint(0,0,0)
		// };
		//
		// configRightScaleSingle = new
		// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
		// Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
		// trajectoryRightScaleSingle = Pathfinder.generate(pointsRightScaleSingle,
		// configRightScaleSingle);
		// modifierRightScaleSingle = new
		// TankModifier(trajectoryRightScaleSingle).modify(0.66);
		//
		// Waypoint[] pointsMiddleRightSingle = new Waypoint[] {
		// new Waypoint(-5.6,0,0),
		// new Waypoint(0,0,0)
		// };
		//
		// configMiddleRightSingle = new
		// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
		// Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
		// trajectoryMiddleRightSingle = Pathfinder.generate(pointsMiddleRightSingle,
		// configMiddleRightSingle);
		// modifierMiddleRightSingle = new
		// TankModifier(trajectoryMiddleRightSingle).modify(0.66);
		// Waypoint[] pointsMiddleLeftSingle = new Waypoint[] {
		// new Waypoint(-1,-3.75,0),
		// new Waypoint(-.5,-3.75,Pathfinder.d2r(30)),
		// new Waypoint(0,0,0),
		// };
		// configMiddleLeftSingle = new
		// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
		// Trajectory.Config.SAMPLES_HIGH, 0.02, 3, 2.3, 12);
		// trajectoryMiddleLeftSingle = Pathfinder.generate(pointsMiddleLeftSingle,
		// configMiddleLeftSingle);
		// modifierMiddleLeftSingle = new
		// TankModifier(trajectoryMiddleLeftSingle).modify(0.66);
		//
		Waypoint[] pointsLeftScaleSingle = new Waypoint[] { new Waypoint(-9.2, -9.5, 0),
				new Waypoint(-0.8, -9.5, Pathfinder.d2r(25)), new Waypoint(0, 0, 0),
				// new Waypoint(0,0,0)
		};

		configLeftScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
		trajectoryLeftScaleSingle = Pathfinder.generate(pointsLeftScaleSingle, configLeftScaleSingle);
		modifierLeftScaleSingle = new TankModifier(trajectoryLeftScaleSingle).modify(0.66);
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// automaster.autoDecider();
		arm.setArmBrake(false);
		drive.resetEncoders();
		gyro.resetGyro();
		drive.shift(true);
		drive.changeBrakeCoast(true);
		liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
		armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
		left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
		right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
		left.configureEncoder(0, 4096 * 2, 0.127);
		right.configureEncoder(0, 4096 * 2, 0.127);
		left.configurePIDVA(2, 0.0, 0.0, 1 / 4.3, 0);
		right.configurePIDVA(2, 0.0, 0.0, 1 / 4.3, 0);
		new RightScaleClose().start();
		// gyro.resetGyro();
		// new DrivetrainPIDCommand(30.0).start();
		// new DistanceTalonPID(180000).start();
		// new LiftMotionProfile(41000).start();
		// logger.setUseParentHandlers(false);
		// ?new RunIntakeWheels(-1).start();
		// new SimpleAuto().start();
		// new GyroPID(90).start();
		// new LiftMotionProfile(36500,2).start();
		// new AngleTalonPID(90).start();
		// new
		// ArmMotionProfile(-3000,ArmSubsystem.ArmStateConstants.INTAKE_FRONT).start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		if (isAutonomous()) {
			// logNewEvent(returnDifferenceInMatchTime() + " " + "I'm In Autonomous Mode!");
		}
		// updateSmartDashboardTesting();
		SmartDashboard.putNumber("arm", arm.getArmRaw());

	}

	@Override
	public void teleopInit() {
		armVelMax = 0;
		intakeTimer.start();
		// gyro.resetGyro();
		// logger.setUseParentHandlers(false);
		// drive.changeBrakeCoast(false);
		isAutoShift = true;
		maxAccelDrive = 0;
		isVPid = true;
		isArmPidRunning = false;
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
		arm.resetEncoder();
		drive.resetEncoders();
		lift.resetLiftEncoder();
		accelTimer.start();
		time.start();
		timeAccel.start();
		rightDTMaxVel = 0;
		leftDTMaxVel = 0;
		isArmPidRunning = false;
		isLiftPidRunning = false;
		isLiftStall = false;
		liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
		customLiftPos = (int) lift.getRawLift();

		customArmPos = (int) arm.getArmRaw();
		armIsFront = true;
		// prev state variables leave at bottom

		// these two are for buttons not the actual
		autoShiftButtonPrevState = false;
		VPidButtonPrevState = false;

		liftManualPrevState = false;
		armManualPrevState = false;
		prevStateFlipArm = false;
		new SetCompressorCommand(true).start();
		arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);
		drive.changeBrakeCoast(false);
		intake.setIntakePiston(false);
		// new Thread(() -> {
		//
		// AxisCamera camera1 =
		// CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axisName,
		// RobotMap.Camera.axisPort);
		// camera1.setResolution(RobotMap.Camera.axisResWidth,
		// RobotMap.Camera.axisResWidth);
		// camera1.setFPS(RobotMap.Camera.axisFPS);
		// AxisCamera camera2 =
		// CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axis2Name,RobotMap.Camera.axis2Port);
		// camera2.setResolution(RobotMap.Camera.axis2ResWidth,
		// RobotMap.Camera.axis2ResHeight);
		// camera2.setFPS(RobotMap.Camera.axis2FPS);
		// AxisCamera camera3 =
		// CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axis3Name,RobotMap.Camera.axis3Port);
		// camera3.setResolution(RobotMap.Camera.axis3ResWidth,
		// RobotMap.Camera.axis3ResHeight);
		// camera3.setFPS(RobotMap.Camera.axis3FPS);
		//
		// CvSink cvSink =
		// CameraServer.getInstance().getVideo(RobotMap.Camera.axisName);
		// CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320,
		// 480);
		//
		// Mat frame = new Mat();
		//
		// while (!Thread.interrupted()) {
		//
		// if (oi.operator.switchToCamera1()) {
		// System.out.println("Switching to camera 1");
		// cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axisName);
		// } else if (oi.operator.switchToCamera2()) {
		// System.out.println("Switching to camera 2");
		// cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axis2Name);
		// } else if (oi.operator.switchToCamera3()) {
		// System.out.println("Switching to camera 3");
		// cvSink = CameraServer.getInstance().getVideo("whiteAxisCamera");
		// cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axis3Name);
		// }
		// if (cvSink.grabFrame(frame) == 0) {
		// continue;
		// }
		//
		// outputStream.putFrame(frame);
		//
		// }
		// }).start();

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//// if (oi.driveJoystickHorizontal.getRawButton(1))
		//// {
		//// logNewEvent(returnDifferenceInMatchTime() + " " + "Button Test!");
		//// }
		// checkAutoShiftToggle();
		// checkVbusToggle();
		//
		// // if(!isAutoShift || oi.driver.forceLowGear()){
		// // //manual shift
		// // }else{
		// // //auto shift
		// // }
		//// if(Math.abs(drive.motors[0].getSelectedSensorVelocity(0)) >
		//// Math.abs(leftDTMaxVel)) {
		//// leftDTMaxVel = drive.motors[0].getSelectedSensorVelocity(0);
		//// }
		//// if(Math.abs(drive.motors[2].getSelectedSensorVelocity(0)) >
		//// Math.abs(leftDTMaxVel)) {
		//// rightDTMaxVel = drive.motors[2].getSelectedSensorVelocity(0);
		//// }
		// SmartDashboard.putNumber("DT Left Max Vel", leftDTMaxVel);
		// SmartDashboard.putNumber("DT Right Max Vel", rightDTMaxVel);
		// if(oi.operatorJoystick.getRawButton(11)){
		// lift.getLiftState();
		//////
		// if (oi.operatorJoystick.getRawButton(2)) {
		// //going up
		// lift.mainLiftMotor.configMotionCruiseVelocity(3200, Robot.timeoutMs);
		// lift.mainLiftMotor.configMotionAcceleration(3650, Robot.timeoutMs); // 400
		//// actual
		// lift.mainLiftMotor.selectProfileSlot(0, 0);
		// SmartDashboard.putNumber("motor current",
		//// lift.mainLiftMotor.getOutputCurrent());
		// //0.3197
		// lift.mainLiftMotor.config_kF(0, 0.307, Robot.timeoutMs);

		// lift.mainLiftMotor.config_kP(0, 5.5, Robot.timeoutMs);
		// lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
		// lift.mainLiftMotor.config_kD(0, 0.05, Robot.timeoutMs);
		// lift.mainLiftMotor.set(ControlMode.MotionMagic, 46000); // something
		// } else if (oi.operatorJoystick.getRawButton(3)) {
		// lift.mainLiftMotor.configMotionCruiseVelocity(4200, Robot.timeoutMs);
		// lift.mainLiftMotor.configMotionAcceleration(4000, Robot.timeoutMs); // 400
		// actual
		// lift.mainLiftMotor.selectProfileSlot(0, 0);
		// //0.3197
		// lift.mainLiftMotor.config_kF(0, 0.3197, Robot.timeoutMs);
		// lift.mainLiftMotor.config_kP(0, 4, Robot.timeoutMs);
		// lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
		// lift.mainLiftMotor.config_kD(0, 0, Robot.timeoutMs);
		// lift.mainLiftMotor.set(ControlMode.MotionMagic, 5500);
		// } else {
		// double liftJoy = oi.operator.getOperatorY();
		// double newLift = liftJoy;
		// if(lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE){
		// if(liftJoy<0){
		// newLift = 0;
		// }
		// }else if(lift.getLiftState() ==
		// LiftSubsystem.LiftHalConstants.CARRIAGE_HIGH_SECOND_HIGH){
		// if(liftJoy>0.185){
		// newLift=0;
		// }
		// }else if(liftJoy == 0){
		// newLift = 0;
		// } else {
		// newLift = liftJoy;
		// }
		// lift.setLift(newLift);
		// SmartDashboard.putNumber("lift encoder", lift.getRawLift());
		// }
		// arm.setArm(0);
		////
		// }else if(oi.operatorJoystick.getRawButton((12))){
		// if(oi.operatorJoystick.getRawButton(2)){
		// if(!isArmPidRunning){
		//// new ArmMotionProfile(-4100).start();
		// }
		// }else if(oi.operatorJoystick.getRawButton(3)){
		// if(!isArmPidRunning){
		//// new ArmMotionProfile(-400).start();
		// }
		// }else{
		// isArmPidRunning = false;
		// double armJoy = oi.operator.getOperatorY();
		// if(armJoy == 0) {
		// if (time.get() > 0.3) {
		// arm.setArmBrake(true);
		// }
		// } else {
		// arm.setArm(armJoy/1.5);
		// arm.setArmBrake(false);
		// time.reset();
		// }
		// }
		//
		// }
		// if(oi.operatorJoystick.getRawButton(9)){
		// intake.setIntakeMotors(1, 1);
		// }else if(oi.operatorJoystick.getRawButton(10)){
		// intake.setIntakeMotors(-1, -1);
		// }else{
		// intake.setIntakeMotors(0, 0);
		// }
		//// if(lift.getLiftState() ==
		// LiftSubsystem.LiftStateConstants.CARRIAGE_LOW_SECOND_MID){
		// if(Math.abs(lift.getRawLiftVel()) > secondStageLiftMaxVel){
		// secondStageLiftMaxVel = Math.abs(lift.getRawLiftVel());
		// }
		//// }else if(lift.getLiftState() ==
		// LiftSubsystem.LiftStateConstants.CARRIAGE_MID_SECOND_HIGH){
		//// if(Math.abs(lift.getRawLiftVel()) > carriageStageMaxVel){
		//// carriageStageMaxVel = Math.abs(lift.getRawLiftVel());
		//// }
		//// }
		// SmartDashboard.putNumber("second stage max vel", secondStageLiftMaxVel);
		//// SmartDashboard.putNumber("carriage max vel", carriageStageMaxVel);
		//// if(Math.abs(Robot.arm.bottomMotor.getSelectedSensorVelocity(0)) > armVelMax
		// && Robot.arm.getArmRaw() > 4950 ){
		//// armVelMax = Math.abs(Robot.arm.bottomMotor.getSelectedSensorVelocity(0));
		//// }
		//// SmartDashboard.putNumber("arm Vel max", armVelMax);
		//// SmartDashboard.putBoolean("is VPID runnig", isVPid
		// // } }
		// SmartDashboard.putNumber("second stage max vel", secondStageLiftMaxVel);
		// SmartDashboard.putNumber("carriage max vel", carriageStageMaxVel);
		// if(Math.abs(Robot.arm.bottomMotor.getSelectedSensorVelocity(0)) > armVelMax
		// && Robot.arm.getArmRaw() > 4950 ){
		// armVelMax = Math.abs(Robot.arm.bottomMotor.getSelectedSensorVelocity(0));
		// }
		// SmartDashboard.putNumber("arm Vel max", armVelMax);
		// SmartDashboard.putBoolean("is VPID runnig", isVPid);
		//
		//// if(oi.operatorJoystick.getRawButton(11)){
		//// arm.setArmBrake(true);
		//// }else if(oi.operatorJoystick.getRawButton(10)){
		//// arm.setArmBrake(false);
		//// }
		////
		////
		//
		// if (timeAccel.get() > 0.05) {
		// timeAccel.stop();
		//// driveVel = drive.motors[0].getSelectedSensorVelocity(0);
		//// driveAccel = (driveVel - prevDriveVel)/timeAccel.get();
		//// prevDriveVel = driveVel;
		// maxLiftVel = (lift.mainLiftMotor.getSelectedSensorVelocity(0) -
		// prevLiftVel)/timeAccel.get();
		// prevLiftVel = lift.mainLiftMotor.getSelectedSensorVelocity(0);
		// timeAccel.reset();
		// timeAccel.start();
		// }
		// SmartDashboard.putNumber("accel of lift", maxLiftVel);
		//// SmartDashboard.putNumber("drive Accleration", driveAccel);
		//// if(oi.operatorJoystick.getRawButton(6)){
		//// intake.setIntakePulse(intakeTimer.get(), 1,false);
		//// }else if(oi.operatorJoystick.getRawButton(4)){
		//// intake.setIntakeMotors(1.0, 1.0);
		//// }else{
		//// intake.setIntakeMotors(0.0, 0.0);
		//// }
		//
		// if(oi.operatorJoystick.getRawButton(1)) {
		// arm.resetEncoder();
		// }
		// SmartDashboard.putNumber("Arm Velocity", arm.getVel());
		// SmartDashboard.putNumber("Arm Angle", arm.getArmAngle());
		//
		//// if (arm.getTime() > 0.05) {
		//// accel = arm.getVel();
		//// SmartDashboard.putNumber("Arm Acceleration", accel-lastAccel);
		//// lastAccel = accel;
		//// arm.time.reset();
		//// }
		//// //
		//// if(oi.operatorJoystick.getRawButton(2)) {
		//// SmartDashboard.putBoolean("is Here", true);
		//// if(!isArmPidRunning){
		//// new ArmMotionProfile(6500).start();
		//// }
		//// SmartDashboard.putNumber("Arm Voltage Motion Magic",
		// arm.bottomMotor.getMotorOutputVoltage());
		//// } else if (oi.operatorJoystick.getRawButton(3)) {
		//// if(!isArmPidRunning){
		//// new ArmMotionProfile(4700).start();
		//// }
		//// SmartDashboard.putNumber("Arm Voltage Motion Magic",
		// arm.bottomMotor.getMotorOutputVoltage());
		//// }else if(!isArmPidRunning) {
		//// isArmPidRunning = false;
		//// double armJoy = oi.operator.getOperatorY();
		//// if(armJoy == 0) {
		//// if (time.get() > 0.3) {
		//// arm.setArmBrake(true);
		//// }
		//// } else {
		//// arm.setArm(armJoy);
		//// arm.setArmBrake(false);
		//// time.reset();
		//// }
		//// SmartDashboard.putBoolean("is Here", false);
		//// }
		// SmartDashboard.putBoolean("is arm pid runnig", isArmPidRunning);
		//// intake.setIntakeMotors(oi.driveJoystickHorizontal.getY(),
		// oi.driveJoystickVertical.getY());
		// drive.shift(true);
		//
		// if(oi.operator.getIntakeForward()){
		// intake.setIntakePiston(true);
		// }else{
		// intake.setIntakePiston(false);
		// }
		//
		// if(lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE){
		// lift.resetLiftEncoder();
		// }
		//// if (oi.operatorJoystick.getPOV() == 0) {
		//// intake.setIntakeMotors(oi.operator.returnSlider(),
		// oi.operator.returnSlider());
		//// } else if (oi.operatorJoystick.getPOV() == 180) {
		//// intake.setIntakeMotors(-oi.operator.returnSlider(),
		// -oi.operator.returnSlider());
		//// } else {
		//// intake.setIntakeMotors(0, 0);
		//// }
		//// if (oi.operatorJoystick.getRawButton(2)) {
		//// drive.motors[0].set(ControlMode.MotionMagic, 23500);
		//// drive.motors[2].set(ControlMode.MotionMagic, 23500);
		////
		//// } else if (oi.operator.getButton4()){
		////// new DistanceTalonPID(180000).start();
		//// }else {
		////// drive.resetEncoders();
		// drive.driveFwdRotate(oi.driver.getForward(), -oi.driver.getRotation(), true);
		//// }
		//
		//
		//// if(!drivePIDRunning){
		//// double joyXVal = -Robot.oi.driver.getRotation();
		//// double joyYVal = Robot.oi.driver.getForward();
		//// if (!isVPid || oi.driver.isVBusOveridePush() || ((Math.abs(joyXVal) < 0.1)
		// && joyYVal == 0)) {
		//// if (joyXVal > 0) {
		//// joyXVal = Math.pow(joyXVal, 2.5);
		//// } else {
		//// joyXVal = -Math.pow(Math.abs(joyXVal), 2.5);
		//// }
		//// SmartDashboard.putBoolean("is in Vbus", true);
		//// drive.driveFwdRotate(joyYVal, joyXVal, true);
		//// } else if (Math.abs(joyXVal) < 0.1 && Math.abs(joyYVal) < 0.15) {
		//// if (joyXVal > 0) {
		//// joyXVal = Math.pow(joyXVal, 2.5);
		//// } else {
		//// joyXVal = -Math.pow(Math.abs(joyXVal), 2.5);
		//// }
		//// drive.driveFwdRotate(joyYVal, joyXVal, true);
		//// } else {
		//// SmartDashboard.putBoolean("is in Vbus", false);
		//// if (joyYVal > 0) {
		//// joyYVal = Math.pow(Math.abs(joyYVal),
		// DrivetrainSubsystem.VPIDConstants.Y_COMPONENT_EXP);
		//// } else {
		//// joyYVal = -Math.pow(Math.abs(joyYVal),
		// DrivetrainSubsystem.VPIDConstants.Y_COMPONENT_EXP);
		//// }
		//// if (joyXVal > 0) {
		//// if (joyXVal > 0.3) {
		//// joyXVal = Math.pow(Math.abs(joyXVal),
		// DrivetrainSubsystem.VPIDConstants.X_COMPONENT_HIGH_EXP);
		//// } else {
		//// joyXVal = Math.pow(Math.abs(joyXVal),
		// DrivetrainSubsystem.VPIDConstants.X_COMPONENT_LOW_EXP);
		//// }
		////
		//// } else {
		//// if (joyXVal < -0.1) {
		//// joyXVal = -Math.pow(Math.abs(joyXVal),
		// DrivetrainSubsystem.VPIDConstants.X_COMPONENT_HIGH_EXP);
		//// } else {
		//// joyXVal = -Math.pow(Math.abs(joyXVal),
		// DrivetrainSubsystem.VPIDConstants.X_COMPONENT_LOW_EXP);
		//// }
		//// }
		//// drive.driveFwdRotate(joyYVal, joyXVal, false);
		//// }
		//// }
		//
		// // drive.rawDrive(oi.driveJoystickVertical.getY(),
		// // oi.driveJoystickVertical.getY());
		// // drive.rawDrive(0.3, 0.3);
		// drive.driveFwdRotate(oi.driver.getForward(), oi.driver.getRotation(), true);
		//
		//
		////
		//// // drive.driveFwdRotate(oi.driver.getForward(), -oi.driver.getRotation(),
		// true);
		//// if (oi.operator.PIDTunePhase()) {
		//// SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
		//// isTuningPID = true;
		//// }
		////
		//// SmartDashboard.updateValues();
		////
		//// }
		// // these are checking the previous state of a variable make sure this is at
		// the
		// // bottom
		// autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		// VPidButtonPrevState = oi.driver.switchToVbus();
		// updateSmartDashboardTesting();
		//
		// drive.rawDrive(0.5, 0.5);
		// if(lidarCount == 12){
		// SmartDashboard.putNumber("Lidar", lidar.getSample());
		// lidarCount = 0;
		// }
		// lidarCount ++;
		teleopRun();

	}

	public void teleopRun() {
		if (oi.driver.shiftUp()) {
			drive.shift(true);
		} else {
			drive.shift(false);
		}
		double rot = -oi.driver.getRotation();
		if (rot > 0) {
			rot = Math.pow(rot, 2);
		} else {
			rot = -Math.pow(Math.abs(rot), 2);
		}
		drive.driveFwdRotate(oi.driver.getForward(), rot, true);

		if (oi.operator.getIntakeState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState).start();
				logNewEvent("Lift in Low State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState).start();
					logNewEvent("Arm towards Battery");
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState).start();
					logNewEvent("Arm towards Diagonals");

				}
			}
		} else if (oi.operator.getStoreState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState).start();
				logNewEvent("Lift in Store State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.STORE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.STORE_REAR) {
				if (armIsFront) {
					timesCalled++;
					SmartDashboard.putNumber("times Call", timesCalled);
					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState).start();
					logNewEvent("Arm in Store Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState).start();
					logNewEvent("Arm in Store Rear State");
				}
			}

		} else if (oi.operator.getSwitchState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState).start();
				logNewEvent("Lift in Switch State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR
					&& armState != ArmSubsystem.ArmStateConstants.SWITCH_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.SWITCH_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, armState).start();
					logNewEvent("Arm in Switch Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR, armState).start();
					logNewEvent("Arm in Switch Rear State");
				}
			}
		} else if (oi.operator.getScaleLowState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE, liftState).start();
				SmartDashboard.putBoolean("got in", true);
				logNewEvent("Lift in Low Scale State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState).start();
					logNewEvent("Arm in Low Scale Pos Arm Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState).start();
					logNewEvent("Arm in Low Scale Pos Arm Rear State");

				}
			}
		} else if (oi.operator.getScaleMidState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE, liftState).start();
				logNewEvent("Lift in Mid Scale State");
				

			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState).start();
					logNewEvent("Arm in Mid Scale Pos Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState).start();
					logNewEvent("Arm in Mid Scale Pos Rear State");

				}
			}
		} else if (oi.operator.getScaleHighState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE, liftState).start();
				logNewEvent("Lift in High Scale State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState).start();
					logNewEvent("Arm in High Scale Pos Arm Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState).start();
					logNewEvent("Arm in High Scale Pos Arm Rear State");

				}
			}

		} else if (oi.operator.getArmUpSmall()) {
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
					&& arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ < 0) {
				armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
				if (arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID) {
					armIsFront = true;
				} else {
					armIsFront = false;
				}
				customArmPos = arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ;
				new ArmMotionProfile(customArmPos, armState).start();
				logNewEvent("Arm Moved Slightly up by "+arm.getArmRaw());

			}
		} else if (oi.operator.getArmDownSmall()) {
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN && arm.getArmRaw()
					- ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.INTAKE_REAR) {
				armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
				if (arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID) {
					armIsFront = true;
				} else {
					armIsFront = false;
				}
				customArmPos = arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ;
				new ArmMotionProfile(customArmPos, armState).start();
				logNewEvent("Arm Moved Slightly Down by "+arm.getArmRaw());
			}
		} else if (oi.operator.getLiftUpSmall()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP && lift.getRawLift()
					+ LiftSubsystem.LiftEncoderConstants.ADJ_DIST < LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP;
				customLiftPos = (int) lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
				new LiftMotionProfile(customLiftPos, liftState).start();
				logNewEvent("Lift Moved Slightly Up by "+lift.getRawLift());
			}

		} else if (oi.operator.getLiftDownSmall()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN
					&& lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST > 0) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN;
				customLiftPos = (int) lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
				new LiftMotionProfile(customLiftPos, liftState).start();
				logNewEvent("Lift Moved Slightly Down by "+lift.getRawLift());
			}
		} else if (oi.operator.getExchangeState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState).start();
				logNewEvent("Lift moved to Exchange Zone State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState).start();
					logNewEvent("Arm moved to Exchange Zone Front State");
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState).start();
					logNewEvent("Arm moved to Exchange Zone Rear State");

				}
			}
		} else if (oi.operator.flipArm()) {
			// if(lift.isCarriageAtBottom()){ //temp
			if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kReverse)) {
				if (!prevStateFlipArm && lift.getRawLift() < LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE) {
					if (armIsFront) {
						if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState).start();
							logNewEvent("Arm Flipped from Exchange Front State to Exchange Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
								|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState)
									.start();
							logNewEvent("Arm Flipped from Custom State to Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState).start();
							logNewEvent("Arm Flipped from High Scale Pos Front State to High Scale Pos Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState).start();
							logNewEvent("Arm Flipped from Mid Scale Pos Front State to Mid Scale Pos Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.STORE_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState).start();
							logNewEvent("Arm Flipped from Store Front State to Store Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState).start();
							logNewEvent("Arm Flipped from Intake Front State to Intake Rear State");
						}
					} else {
						if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState).start();
							logNewEvent("Arm Flipped from Exchange Rear State to Exchange Front State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN
								|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState)
									.start();
							logNewEvent("Arm Flipped from Custom Up State to Intake Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState).start();
							logNewEvent("Arm Flipped from High Scale Rear State to High Scale Front State");
						} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState).start();
							logNewEvent("Arm Flipped from Mid Scale Rear State to Mid Scale Front State");

						} else if (armState == ArmSubsystem.ArmStateConstants.STORE_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState).start();
							logNewEvent("Arm Flipped from Store Rear State to Store Front State");

						} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState).start();
							logNewEvent("Arm Flipped from Intake Rear State to Intake Front State");						}
					}
				}
			}

			// }
		} else if (oi.operator.isManual()) {
			if(enteredManualMode == false)
			{
				logNewEvent("MANUAL MODE ENTERED, LOGGING STOPPED!");
				enteredManualMode = true;
			}
			liftState = LiftSubsystem.LiftStateConstants.CUSTOM_STATE;
			armState = ArmSubsystem.ArmStateConstants.CUSTOM;
			customLiftPos = (int) lift.getRawLift();
			customArmPos = (int) arm.getArmRaw();
			double liftJoy = oi.operator.getManualLift();
			double newLift = liftJoy;
			if (lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE) {
				if (liftJoy < 0) {
					newLift = 0;
				}
				liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;

			} else if (lift.getLiftState() == LiftSubsystem.LiftHalConstants.CARRIAGE_HIGH_SECOND_HIGH) {
				if (liftJoy > 0.185) {
					newLift = 0.185;
				}
			} else if (liftJoy == 0) {
				newLift = 0.185;
			} else {
				newLift = liftJoy;
			}
			lift.setLift(newLift);
			double armJoy = oi.operator.getManualArm();
			if (armJoy == 0) {
				if (time.get() > 0.3) {
					arm.setArmBrake(true);
				}
			} else {
				arm.setArm(armJoy / 1.5);
				arm.setArmBrake(false);
				time.reset();
			}
		} else if (oi.operatorJoystick.getRawButton(7)) {
			new FlipRaisedArm(1).start();
		} else {
			if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				lift.setLift(0);
			} else if (liftState == LiftSubsystem.LiftStateConstants.SWITCH_STATE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.SWITCH_STATE);
			} else if (liftState == LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE);
			} else if (liftState == LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE);
			} else if (liftState == LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE);
			} else if (liftState == LiftSubsystem.LiftStateConstants.CUSTOM_STATE) {
				lift.setLiftMotion(customLiftPos);
			}
			if (!isArmPidRunning) {
				arm.setArmBrake(true);
			}
		}
		if (oi.operator.deployOnlyWheels()) {
			new RunIntakeWheels(-1).start();
		} else if (oi.operator.deployWithWheelsAndOpen()) {
			new DeployWithWheelsAndIntake().start();
			;
		} else if (oi.operator.openIntake()) {
			new SetIntakePistons(true).start();
		} else if (oi.operator.runIntakeWithWheelsClosed()) {
			new IntakeWithWheelsAndClose().start();
		} else if (oi.operator.closeIntake()) {
			new SetIntakePistons(false).start();
		} else {
			intake.setIntakeMotors(0, 0);
		}
		prevStateFlipArm = oi.operator.flipArm();
		if (arm.getArmRaw() > (ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT
				+ ArmSubsystem.ArmEncoderConstants.INTAKE_REAR) / 2) {
			armIsFront = true;
		} else {
			armIsFront = false;
		}
		if (lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE) {
			lift.resetLiftEncoder();
		}
		SmartDashboard.putNumber("State", liftState);
		SmartDashboard.putNumber("arm state", armState);
		SmartDashboard.putBoolean("isarmfront", armIsFront);
		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
		SmartDashboard.putNumber("arm raw", arm.getArmRaw());
		SmartDashboard.putBoolean("isCarriageAtBot", lift.isCarriageAtBottom());
		if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kForward)) {
			SmartDashboard.putString("intake", "forw");
		} else if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kReverse)) {
			SmartDashboard.putString("intake", "rev");

		}
		if (lift.isSecondStageAtBottom()) {
			lidarOffset = lidar.getSample();
		}
		if (lidarCount == 12) {
			lidarValue = lidar.getSample() - lidarOffset;
			lidarCount = 0;
		}
		lidarCount++;
		SmartDashboard.putNumber("adj Lidar", lidarValue);
		SmartDashboard.putBoolean("Can Flip", lift.canFlip());
	}

	private void checkAutoShiftToggle() {
		// on release
		if (!oi.driver.switchToNormalShift() && autoShiftButtonPrevState) {
			isAutoShift = !isAutoShift;
		}
	}

	private void checkVbusToggle() {
		if (!oi.driver.switchToVbus() && VPidButtonPrevState) {
			isVPid = !isVPid;
		}
	}

	private void updateSmartDashboardTesting() {
		SmartDashboard.putNumber("Gyro Val", gyro.getGyroAngle());
		SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
		SmartDashboard.putNumber("k_p", k_p);
		SmartDashboard.putNumber("k_i", k_i);
		SmartDashboard.putNumber("k_d", k_d);
		SmartDashboard.putNumber("ARM ENCODER", arm.bottomMotor.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("TalonRaw",
		// drive.motors[0].getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Talon Enc Distance", drive.getTalonDistanceLeft());
		// SmartDashboard.putBoolean("Infrared", arm.getInfraredSensor());
		SmartDashboard.putNumber("Talon Enc Distance Left", drive.motors[0].getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Talon Enc Distance Right", drive.motors[2].getSelectedSensorPosition(0));
		SmartDashboard.putNumber("LiftJoy", oi.operator.getOperatorY());
		SmartDashboard.putNumber("WINCH RPM", lift.mainLiftMotor.getSelectedSensorVelocity(0));

		if (accelTimer.get() > 0.1) {
			SmartDashboard.putNumber("drive accel",
					(drive.motors[0].getSelectedSensorVelocity(0) - prevDriveVel) / accelTimer.get());
			SmartDashboard.putNumber("winch accel",
					(lift.mainLiftMotor.getSelectedSensorVelocity(0) - prevWinchVel) / accelTimer.get());
			accelTimer.reset();
			prevWinchVel = lift.mainLiftMotor.getSelectedSensorVelocity(0);
			prevDriveVel = drive.motors[0].getSelectedSensorVelocity(0);
		}
		// SmartDashboard.putNumber("PID error", drive.motors[2].getClosedLoopError(0));
		// SmartDashboard.putNumber("Target Current output",
		// drive.motors[2].getMotorOutputPercent());
		//
		// SmartDashboard.putNumber("Left Velocity",
		// drive.motors[0].getSelectedSensorVelocity(0));
		// SmartDashboard.putNumber("Left Dist",
		// drive.motors[0].getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("Right Velocity",
		// drive.motors[2].getSelectedSensorVelocity(0));
		// SmartDashboard.putNumber("Right Dist",
		// drive.motors[2].getSelectedSensorPosition(0));
		// SmartDashboard.putBoolean("Bottom Second Stage Hal",
		// !lift.botSecondStageHal.get());
		// SmartDashboard.putBoolean("Bottom Carriage Stage Hal",
		// !lift.botCarriageHal.get());
		// SmartDashboard.putBoolean("Top Second Stage Hal",
		// !lift.topSecondStageHal.get());
		// SmartDashboard.putBoolean("Top Carriage Hal", !lift.topCarriageHal.get());
		// SmartDashboard.putNumber("Carriage Winch Raw", lift.getRawLift());
		SmartDashboard.putBoolean("is Second Stage at Bottom", lift.isSecondStageAtBottom());
		SmartDashboard.putBoolean("is Second Stage at Top", lift.isSecondStageAtTop());
		SmartDashboard.putBoolean("is Carriage at Bottom", lift.isCarriageAtBottom());
		SmartDashboard.putBoolean("is Carriage at Top", lift.isCarriageAtTop());
		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
		SmartDashboard.putNumber("Lift Scaled Distance", lift.getLiftDistance());
		SmartDashboard.putNumber("Arm Raw", arm.getArmRaw());
		SmartDashboard.putNumber("Vel arm", arm.bottomMotor.getSelectedSensorVelocity(0));
		// SmartDashboard.putNumber("Arm scaled", arm.getArmPosition());
		if (lidarCount == 12) {
			SmartDashboard.putNumber("Lidar", lidar.getSample());
			lidarCount = 0;
		}
		// SmartDashboard.putNumber("Current 11", PDPJNI.getPDPChannelCurrent(11, 0));
		lidarCount++;
		// SmartDashboard.putNumber("curr", drive.motors[0].getOutputCurrent());
		// SmartDashboard.putNumber("pDP", p.getTotalCurrent());
		int state = lift.getLiftState();
		if (state == 1) {
			SmartDashboard.putString("Lift State (carriage/second)", "Low Low");
		} else if (state == 2) {
			SmartDashboard.putString("Lift State (carriage/second)", "Low Mid");
		} else if (state == 3) {
			SmartDashboard.putString("Lift State (carriage/second)", "Low High");
		} else if (state == 4) {
			SmartDashboard.putString("Lift State (carriage/second)", "Mid High");
		} else if (state == 5) {
			SmartDashboard.putString("Lift State (carriage/second)", "High High");
		} else if (state == 6) {
			SmartDashboard.putString("Lift State (carriage/second)", "High Mid");
		} else if (state == 7) {
			SmartDashboard.putString("Lift State (carriage/second)", "High Low");
		} else if (state == 8) {
			SmartDashboard.putString("Lift State (carriage/second)", "Mid Low");
		}
	}

	private void updateSmartDashboardComp() {

	}

	public void logNewEvent(String eventToLog) {
		logger.info(returnDifferenceInMatchTime()+" "+eventToLog);
	}

	public String returnDifferenceInMatchTime() {
		if (DriverStation.getInstance().isAutonomous()) {
			modeTime = autoTime;
			currentTime = DriverStation.getInstance().getMatchTime();
		}
		if (DriverStation.getInstance().isOperatorControl()) {
			modeTime = teleOpTime;
			currentTime = DriverStation.getInstance().getMatchTime();
		}
		conversionTime = Double.toString(modeTime - currentTime);
		return conversionTime;
	}

}
