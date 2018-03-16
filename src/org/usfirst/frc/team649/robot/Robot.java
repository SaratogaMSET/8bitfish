
package org.usfirst.frc.team649.robot;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.autonomous.AutoTest;
import org.usfirst.frc.team649.autonomous.CenterSwitchLeft;
import org.usfirst.frc.team649.autonomous.CenterSwitchRight;
import org.usfirst.frc.team649.autonomous.LeftScale;
import org.usfirst.frc.team649.autonomous.RightFarScale;
import org.usfirst.frc.team649.autonomous.RightScale;
import org.usfirst.frc.team649.autonomous.RightSwitch;
import org.usfirst.frc.team649.autonomous.autoMaster;
import org.usfirst.frc.team649.robot.RobotMap.AutoSwitches;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.CommandGroups.FlipRaisedArm;
import org.usfirst.frc.team649.robot.CommandGroups.IntakeWithWheelsAndClose;
import org.usfirst.frc.team649.robot.CommandGroups.RightScaleClose;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.Diagnostic;
import org.usfirst.frc.team649.robot.commands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.DistanceTalonPID;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfile;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.GyroStraightPID;

import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetCompressorCommand;
import org.usfirst.frc.team649.robot.commands.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.SimpleAuto;
import org.usfirst.frc.team649.robot.commands.ZeroArmRoutine;

import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.AutoSelector;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.robot.util.Lidar;
import org.usfirst.frc.team649.test.AutoTestCommand;
import org.usfirst.frc.team649.test.Square;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc.team649.robot.util.CameraServer;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends TimedRobot {

	public static OI oi;
	public static boolean canFlipArm;
	public static boolean isPIDActive;
	public static boolean isAutoShift;
	public static boolean isVPid;
	public static boolean isHigh;
	public static boolean isTuningPID = true;
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
	public static AutoTest autoTest;
	public static AutoSelector switches;
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
	public static boolean isZero;
	public Timer time;
	public Timer timeAccel;

	public static boolean isArmPidRunning;
	public static boolean isDrivePIDRunning;	


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

	public static double robotLength = 32; // idk
	public double lidarOffset;

	public static boolean isLiftPidRunning;
	public static boolean isLiftStall;
	public static int liftState;
	public static int liftHalState;
	public static int customLiftPos;
	int timesCalled;
	public static int armState;
	public static int customArmPos;
	public static boolean armIsFront;
	public static boolean isTestingAuto = true;
	public static int program = 1;
	

	public static boolean isOpen;
	public static boolean isMPRunning;

	public static EncoderFollower left;
	public static EncoderFollower right;

	public static Trajectory.Config configRightScaleSingle;
	public static Trajectory trajectoryRightScaleSingle;
	public static TankModifier modifierRightScaleSingle;

	public static Trajectory.Config configRightScaleSingle2;
	public static Trajectory trajectoryRightScaleSingle2;
	public static TankModifier modifierRightScaleSingle2;

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
	public boolean hasEndgameStarted;
	public String Alliance = "Red";
	public static I2C arduino;

	@Override

	public void robotInit() {
		useLogging = false;
		drivePIDRunning = false;
		prevWinchVel = 0;
		oi = new OI();
		lidarCount = 0;
		drive = new DrivetrainSubsystem();
		gyro = new GyroSubsystem();
		arm = new ArmSubsystem();
//		switches = new AutoSelector();
		intake = new IntakeSubsystem();
		intakeTimer = new Timer();
		lift = new LiftSubsystem();
		prevLiftVel = 0;
		maxLiftVel = 0;
		// automaster = new autoMaster();
		lidar = new Lidar(I2C.Port.kOnboard, 0xC4 >> 1);
		compressor = new Compressor(4);
		isPIDActive = false;
		autoTest = new AutoTest();
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
		isZero = false;
		time = new Timer();
		timeAccel = new Timer();
		driveAccel = 0;
		driveVel = 0;
		canFlipArm = false;
		prevDriveVel = 0;
		liftState = 2;
		if (arm.getArmRaw() < ArmSubsystem.ArmEncoderConstants.INTAKE_REAR / 2) {
			armState = ArmSubsystem.ArmStateConstants.INTAKE_REAR;
		} else {
			armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
		}
		rightDTMaxVel = 0;
		leftDTMaxVel = 0;
		timesCalled = 0;
		lidarOffset = 0;
		lidarValue = lidar.getSample();

		isOpen = false;
		isMPRunning = false;
//		arduino = new I2C(I2C.Port.kOnboard, 0x2);

		Waypoint[] pointsRightScaleSingle = new Waypoint[] { new Waypoint(-12.9, -2.9, 0), new Waypoint(-7, -2.9, 0),
				new Waypoint(-4, -2.9, Pathfinder.d2r(45)), new Waypoint(-2.05, 0, 0), new Waypoint(0, 0, 0) };

		configRightScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
		trajectoryRightScaleSingle = Pathfinder.generate(pointsRightScaleSingle, configRightScaleSingle);
		modifierRightScaleSingle = new TankModifier(trajectoryRightScaleSingle).modify(0.66);
		//
		// Waypoint[] pointsRightScaleSingle2 = new Waypoint[] {
		// new Waypoint(-12.9,-2.9,0),
		// new Waypoint(-7,-2.9,0),
		// new Waypoint(-4,-2.9,Pathfinder.d2r(45)),
		// new Waypoint(-2.05,0,0),
		// new Waypoint(0,0,0)
		// };
		//
		// configRightScaleSingle2 = new
		// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
		// Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
		// trajectoryRightScaleSingle2 =
		// Pathfinder.generate(pointsRightScaleSingle2,
		// configRightScaleSingle2);
		// modifierRightScaleSingle2 = new
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
		// trajectoryMiddleRightSingle =
		// Pathfinder.generate(pointsMiddleRightSingle,
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
		// trajectoryMiddleLeftSingle =
		// Pathfinder.generate(pointsMiddleLeftSingle, configMiddleLeftSingle);
		// modifierMiddleLeftSingle = new
		// TankModifier(trajectoryMiddleLeftSingle).modify(0.66);
		//
		// Waypoint[] pointsLeftScaleSingle = new Waypoint[] {
		// new Waypoint(-9.9,-8.3,0),
		// new Waypoint(-1.1,-8.3,Pathfinder.d2r(45)),
		// new Waypoint(0,0,0),
		// };
		//
		// configLeftScaleSingle = new
		// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
		// Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
		// trajectoryLeftScaleSingle =
		// Pathfinder.generate(pointsLeftScaleSingle, configLeftScaleSingle);
		// modifierLeftScaleSingle = new
		// TankModifier(trajectoryLeftScaleSingle).modify(0.66);

		//
		// File leftScaleSingle = new
		// File("C:\\Users\\MSET\\workspace\\leftScaleSingle.csv");
		// Pathfinder.writeToCSV(leftScaleSingle,
		// modifierLeftScaleSingle.getLeftTrajectory());
		//
//			logger = Logger.getLogger("robotLog");
//			matchTimer = new Timer();
//			logLocationString = "/media/sda1/" + DriverStation.getInstance().getEventName().trim() + "/match number "
//					+ DriverStation.getInstance().getMatchNumber() + ".txt";
//			logLocation = new File(logLocationString);
//
//			try {
//				usb = new FileHandler(logLocationString);
//				logger.addHandler(usb);
//				SimpleFormatter usbFormatter = new SimpleFormatter();
//				usb.setFormatter(usbFormatter);
//			} catch (SecurityException e) {
//				e.printStackTrace();
//			} catch (IOException e) {
//				e.printStackTrace();
//			}
//			if (Alliance.equals("Blue")) {
//				UpdateLEDs("a:b");
//			} else if (Alliance.equals("Red")) {
//				UpdateLEDs("a:r");
//			} else {
//				UpdateLEDs("a:w");
//			}
//		arduino = new I2C(I2C.Port.kMXP, 0xA);

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
		// trajectoryRightScaleSingle =
		// Pathfinder.generate(pointsRightScaleSingle,
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
		// trajectoryMiddleRightSingle =
		// Pathfinder.generate(pointsMiddleRightSingle,
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
		// trajectoryMiddleLeftSingle =
		// Pathfinder.generate(pointsMiddleLeftSingle,
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

//		if (Alliance == "Blue") {
//			UpdateLEDs("a:b");
//		} else if (Alliance == "Red") {
//			UpdateLEDs("a:r");
//		} else {
//			UpdateLEDs("a:w");
//		}
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
//		automaster.autoDecider();
//		arm.setArmBrake(true);
//		drive.resetEncoders();
////		new DrivetrainMotionProfileIn(50.0).start();
//		liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
//		drive.shift(true);
//		for(int i = 0; i < 4; i++) {
//			drive.motors[i].setNeutralMode(NeutralMode.Brake);
//			drive.motors[i].configMotionAcceleration(9000, timeoutMs);
//			drive.motors[i].configMotionCruiseVelocity(18000, timeoutMs);
//			drive.motors[i].config_kP(0, 1.5, Robot.timeoutMs);
//			drive.motors[i].config_kI(0, 0, Robot.timeoutMs);
//			drive.motors[i].config_kD1(0, 0.9, Robot.timeoutMs);
//		}
//		drive.changeBrakeCoast(true);
//		if(program == 1) {
//			new CenterSwitchRight().start();
//		} else if (program == 2) {
//			new CenterSwitchLeft().start();
//		}
//		new LeftScale().start();
//		new RightSwitch().start();
//		new DrivetrainMotionProfileIn(230).start();
//		new GyroStraightPID(200).start();
//		new GyroPID(90).start();
//		int liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
//		new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE,liftState).start();
		// automaster.autoDecider();
		isZero = false;
		// automaster.autoDecider();

		arm.setArmBrake(false);
		drive.resetEncoders();
		gyro.resetGyro();
		drive.shift(true);
		drive.changeBrakeCoast(true);
		new ZeroArmRoutine().start();
		// new
		// ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE).start();

//		liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
		// armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
		// File myFileL = new
		// File("C:\\Users\\MSET\\Downloads\\Motion_Profile_Generator-1.0.2\\Motion_Profile_Generator-1.0.2\\images\\Paht.csv_left_detailed.csv");
		// Trajectory trajectoryL = Pathfinder.readFromCSV(myFileL);
		//
		// File myFileR = new
		// File("C:\\Users\\MSET\\Downloads\\Motion_Profile_Generator-1.0.2\\Motion_Profile_Generator-1.0.2\\images\\Paht.csv_right_detailed.csv");
		// Trajectory trajectoryR = Pathfinder.readFromCSV(myFileR);
		left = new EncoderFollower(modifierRightScaleSingle.getLeftTrajectory());
		right = new EncoderFollower(modifierRightScaleSingle.getRightTrajectory());
		left.configureEncoder(0, 4096 * 2, 0.127);
		right.configureEncoder(0, 4096 * 2, 0.127);
		left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
		right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
		new RightScaleClose().start();
//		new ChangeRobotLiftState(10).start();
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
//		armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
//		left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
//		right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
//		left.configureEncoder(0, 4096 * 2, 0.127);
//		right.configureEncoder(0, 4096 * 2, 0.127);
//		left.configurePIDVA(2, 0.0, 0.0, 1 / 4.3, 0);
//		right.configurePIDVA(2, 0.0, 0.0, 1 / 4.3, 0);
//		new RightScaleClose().start();
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
			// logNewEvent(returnDifferenceInMatchTime() + " " + "I'm In
			// Autonomous Mode!");
		}
		// updateSmartDashboardTesting();
		SmartDashboard.putNumber("arm", arm.getArmRaw());
		updateSmartDashboardTesting();
		SmartDashboard.putNumber("Lift state", liftState);


	}

	@SuppressWarnings("unused")
	@Override
	public void teleopInit() {
		armVelMax = 0;
		intakeTimer.start();
		isZero = true;
		// gyro.resetGyro();
		// logger.setUseParentHandlers(false);
		drive.changeBrakeCoast(false);
		// gyro.resetGyro();
		// logger.setUseParentHandlers(false);
		// drive.changeBrakeCoast(false);
		isAutoShift = true;
		maxAccelDrive = 0;
		isVPid = true;
		isArmPidRunning = false;

		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
//		arm.resetEncoder();

		// autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		// VPidButtonPrevState = oi.driver.switchToVbus();
		// arm.resetEncoder();

		drive.resetEncoders();
		// lift.resetLiftEncoder();
		accelTimer.start();
		// if(arm.getArmRaw() < ArmSubsystem.ArmEncoderConstants.INTAKE_REAR/2){
		// armState = ArmSubsystem.ArmStateConstants.INTAKE_REAR;
		// }else{
		// armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
		// }
		time.start();
		timeAccel.start();
		rightDTMaxVel = 0;
		leftDTMaxVel = 0;

//		new SetCompressorCommand(true).start();
		SmartDashboard.putBoolean("Diag?", false);
//		arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);
		isDrivePIDRunning = false;
//		new Diagnostic().start();
		isArmPidRunning = false;
		isLiftPidRunning = false;
		isLiftStall = false;
		// liftState =
		// LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;

		liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
		customLiftPos = (int) lift.getRawLift();

		customArmPos = (int) arm.getArmRaw();
		isOpen = false;
		new SetIntakePistons(false, true).start();
		if (arm.getArmRaw() < ArmSubsystem.ArmEncoderConstants.INTAKE_REAR / 2) {
			armIsFront = false;
		} else {
			armIsFront = true;
		} // prev state variables leave at bottom

		// these two are for buttons not the actual
		autoShiftButtonPrevState = false;
		VPidButtonPrevState = false;

		liftManualPrevState = false;
		armManualPrevState = false;
		prevStateFlipArm = false;
//		new SetCompressorCommand(true).start();
		arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);
		drive.changeBrakeCoast(true);
		for(int i = 0; i < 4; i++) {
			drive.motors[i].setNeutralMode(NeutralMode.Brake);
		}
		drive.shift(true);
//		 new AutoTestCommand().start();
//		 drive.shift(true);
			for(int i = 0; i < 4; i++) {
				drive.motors[i].setNeutralMode(NeutralMode.Brake);
				drive.motors[i].configMotionAcceleration(9000, timeoutMs);
				drive.motors[i].configMotionCruiseVelocity(18000, timeoutMs);
				drive.motors[i].config_kP(0, 1.5, Robot.timeoutMs);
				drive.motors[i].config_kI(0, 0, Robot.timeoutMs);
				drive.motors[i].config_kD(0, 0.9, Robot.timeoutMs);
			}
		drive.changeBrakeCoast(false);
//	   new Thread(() -> {
//
//        AxisCamera camera3 = CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axis3Name,
//                RobotMap.Camera.axis3Port);
//        camera3.setResolution(RobotMap.Camera.axis3ResWidth, RobotMap.Camera.axis3ResWidth);
//        camera3.setFPS(RobotMap.Camera.axis3FPS);
//        AxisCamera camera6 = CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axis6Name,
//                RobotMap.Camera.axis6Port);
//        camera6.setResolution(RobotMap.Camera.axis6ResWidth, RobotMap.Camera.axis6ResHeight);
//        camera6.setFPS(RobotMap.Camera.axis6FPS);
//
//        CvSink cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axis6Name);
//        CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 480);
//
//        Mat frame = new Mat();
//
//        while (!Thread.interrupted()) {
//
//            if (oi.operator.switchToCamera4()) {
//                System.out.println("Switching to camera 1");
//                cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axis3Name);
//            } else if (oi.operator.switchToCamera5()) {
//                System.out.println("Switching to camera 2");
//                cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axis6Name);
//            }
//            if (cvSink.grabFrame(frame) == 0) {
//                continue;
//            }
//            outputStream.putFrame(frame);
//
//        }
//    }).start();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
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
			rot = Math.pow(rot, 1);
		} else {
			rot = -Math.pow(Math.abs(rot), 1);
		}
		drive.driveFwdRotate(oi.driver.getForward(), rot, true);

		if (oi.operator.getIntakeState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState,0).start();
				//logNewEvent("Lift in Low State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState).start();
					//logNewEvent("Arm towards Battery");
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState).start();
					//logNewEvent("Arm towards Diagonals");

				}
			}
		} else if (oi.operator.getStoreState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState,0).start();
				//logNewEvent("Lift in Store State");
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
					//logNewEvent("Arm in Store Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState).start();
					//logNewEvent("Arm in Store Rear State");
				}
			}

		} else if (oi.operator.getSwitchState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState,0).start();
				//logNewEvent("Lift in Switch State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR
					&& armState != ArmSubsystem.ArmStateConstants.SWITCH_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.SWITCH_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, armState).start();
					//logNewEvent("Arm in Switch Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR, armState).start();
					//logNewEvent("Arm in Switch Rear State");
				}
			}
		} else if (oi.operator.getScaleLowState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE, liftState,0).start();
				SmartDashboard.putBoolean("got in", true);
				//logNewEvent("Lift in Low Scale State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState).start();
					//LogNewEvent("Arm in Low Scale Pos Arm Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState).start();
					//LogNewEvent("Arm in Low Scale Pos Arm Rear State");

				}
			}
		} else if (oi.operator.getScaleMidState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE, liftState,0).start();
				//LogNewEvent("Lift in Mid Scale State");

			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState).start();
					//LogNewEvent("Arm in Mid Scale Pos Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState).start();
					//LogNewEvent("Arm in Mid Scale Pos Rear State");

				}
			}
		} else if (oi.operator.getScaleHighState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE, liftState,0).start();
				//LogNewEvent("Lift in High Scale State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState).start();
					//LogNewEvent("Arm in High Scale Pos Arm Front State");

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState).start();
					//LogNewEvent("Arm in High Scale Pos Arm Rear State");

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
				//LogNewEvent("Arm Moved Slightly up by " + arm.getArmRaw());

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
				//LogNewEvent("Arm Moved Slightly Down by " + arm.getArmRaw());
			}
		} else if (oi.operator.getLiftUpSmall()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP && lift.getRawLift()
					+ LiftSubsystem.LiftEncoderConstants.ADJ_DIST < LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP;
				customLiftPos = (int) lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
				new LiftMotionProfile(customLiftPos, liftState,0).start();
				//LogNewEvent("Lift Moved Slightly Up by " + lift.getRawLift());
			}

		} else if (oi.operator.getLiftDownSmall()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN
					&& lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST > 0) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN;
				customLiftPos = (int) lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
				new LiftMotionProfile(customLiftPos, liftState,0).start();
				//LogNewEvent("Lift Moved Slightly Down by " + lift.getRawLift());
			}
		} else if (oi.operator.getExchangeState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_2) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE, liftState,0).start();
				//LogNewEvent("Lift moved to Exchange Zone State");
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState).start();
					//LogNewEvent("Arm moved to Exchange Zone Front State");
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState).start();
					//LogNewEvent("Arm moved to Exchange Zone Rear State");

				}
			}

		} else if (oi.operator.flipArm()) {
			// if(lift.isCarriageAtBottom()){ //temp
			if (liftState % 2 == 1) {
				canFlipArm = false;
			} else {
				canFlipArm = lift.canFlip();
			}
			canFlipArm = true;
			if (!isOpen && canFlipArm) {
				if (!prevStateFlipArm && lift.getRawLift() < LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE) {
					if (armIsFront) {
						if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState).start();
							//LogNewEvent("Arm Flipped from Exchange Front State to Exchange Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
								|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState)
									.start();
							//LogNewEvent("Arm Flipped from Custom State to Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState).start();
							//LogNewEvent("Arm Flipped from High Scale Pos Front State to High Scale Pos Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState).start();
							//LogNewEvent("Arm Flipped from Mid Scale Pos Front State to Mid Scale Pos Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.STORE_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState).start();
							//LogNewEvent("Arm Flipped from Store Front State to Store Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
							//LogNewEvent("Arm Flipped from Intake Front State to Intake Rear State");

							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState).start();
						} else if (armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR, armState).start();
							//LogNewEvent("Arm Flipped from Intake Front State to Intake Rear State");

						}
					} else {
						if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState).start();
							//LogNewEvent("Arm Flipped from Exchange Rear State to Exchange Front State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN
								|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState)
									.start();
							//LogNewEvent("Arm Flipped from Custom Up State to Intake Rear State");
						} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState).start();
							//LogNewEvent("Arm Flipped from High Scale Rear State to High Scale Front State");
						} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState).start();
							//LogNewEvent("Arm Flipped from Mid Scale Rear State to Mid Scale Front State");

						} else if (armState == ArmSubsystem.ArmStateConstants.STORE_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState).start();
							//LogNewEvent("Arm Flipped from Store Rear State to Store Front State");

						} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;

							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState).start();
							//LogNewEvent("Arm Flipped from Intake Rear State to Intake Front State");
						} else if (armState == ArmSubsystem.ArmStateConstants.SWITCH_REAR
								|| armState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR) {
							armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, armState).start();
							//LogNewEvent("Arm Flipped from Switch Rear State to Intake Front State");
						}
					}
				}
			}

			// }
		} else if (oi.operator.isManual()) {
			if (enteredManualMode == false) {
				//LogNewEvent("MANUAL MODE ENTERED, LOGGING STOPPED!");
				enteredManualMode = true;
			}
			liftState = LiftSubsystem.LiftStateConstants.CUSTOM_STATE;
			armState = ArmSubsystem.ArmStateConstants.CUSTOM;
			customLiftPos = (int) lift.getRawLift();
			customArmPos = (int) arm.getArmRaw();
			// double liftJoy = oi.operator.getManualLift();
			// double newLift = liftJoy;
			// if(lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE){
			// if(liftJoy<0){
			// newLift = 0;
			// }
			// liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
			//
			// }else if(lift.getLiftState() ==
			// LiftSubsystem.LiftHalConstants.CARRIAGE_HIGH_SECOND_HIGH){
			// if(liftJoy>0.185){
			// newLift=0.15;
			// }
			// }else if(liftJoy == 0){
			// newLift = 0.15;
			// }else {
			// newLift = liftJoy;
			// }
			// lift.setLift(newLift);

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
				if (lift.getLiftState() != LiftSubsystem.LiftHalConstants.LOWEST_STATE) {
					lift.setLift(0.15);
				} else {
					lift.setLift(0);
				}
			}else if(liftState == LiftSubsystem.LiftStateConstants.INTAKE_2){
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE);
			}
			if (!isArmPidRunning) {
				arm.setArmBrake(true);
				// if(armState == ArmSubsystem.)
			}

		}
		// if(oi.operator.deployOnlyWheels()){
		// new RunIntakeWheels(-1).start();
		// }else if(oi.operator.deployWithWheelsAndOpen()){
		// new DeployWithWheelsAndIntake().start();
		// }else if(oi.operator.openIntake()){
		// new SetIntakePistons(true,false).start();
		// }else if(oi.operator.runIntakeWithWheelsClosed()){
		// new IntakeWithWheelsAndClose().start();
		// }else if(oi.operator.closeIntake()){
		// new SetIntakePistons(false,false).start();
		// }else{
		// intake.setIntakeMotors(0, 0);
		//// }
		if (armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT
				|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {

		} else if (oi.operator.deployOnlyWheels()) {
			// if(armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT|| armState ==
			// ArmSubsystem.ArmStateConstants.SWITCH_REAR){
			// new RunIntakeWheels(-0.6);
			// }else{
			new RunIntakeWheels(-0.65).start();
			// }
		} else if (oi.operator.openIntake()) {
			isOpen = true;
			new SetIntakePistons(true, false).start();
			Robot.intake.setIntakeMotors(0, 0);

		} else if (oi.operator.runIntakeWithWheelsClosed()) {
			isOpen = false;
			new IntakeWithWheelsAndClose().start();
		} else if (oi.operator.closeIntake()) {
			isOpen = false;
			new SetIntakePistons(false, true).start();
			Robot.intake.setIntakeMotors(0, 0);

		} else if (isOpen == false) {
			new SetIntakePistons(false, true).start();
			Robot.intake.setIntakeMotors(0, 0);

		} else {
			Robot.intake.setIntakeMotors(0, 0);
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
//		SmartDashboard.putNumber("pot 1", switches.getPot1());
//		SmartDashboard.putNumber("pot 2", switches.getPot2());
//		SmartDashboard.putNumber("pot 3", switches.getPot3());
//		SmartDashboard.putNumber("pot 4", switches.getPot4());
//		SmartDashboard.putNumber("pot side", switches.getPotSide());

		if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kForward)) {
			SmartDashboard.putString("intake", "forw");
		} else if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kReverse)) {
			SmartDashboard.putString("intake", "rev");
		}

		if (arm.getArmHalZeroFront()) {
			arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);
		} else if (arm.getArmHalZeroBack()) {
			arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - 5, 0, 20);
		}
		if (lift.isSecondStageAtBottom()) {
			lidarOffset = lidar.getSample();
		}
		if (lidarCount == 6) {
			lidarValue = lidar.getSample() - lidarOffset;
			lidarCount = 0;
		}
		lidarCount++;
		SmartDashboard.putNumber("adj Lidar", lidarValue);
		SmartDashboard.putBoolean("Can Flip", lift.canFlip());
		SmartDashboard.putNumber("Arm Current", arm.bottomMotor.getOutputCurrent());
//		if (DriverStation.getInstance().getMatchTime() <= 30.0 && DriverStation.getInstance().getMatchTime() >= 29.0) {
//			hasEndgameStarted = true;
//			if (hasEndgameStarted) {
//				UpdateLEDs("e:s");
//				hasEndgameStarted = false;
//			}
//		}
//		if (intake.isRunning() == true) {
//			if (arm.getInfraredSensor() == true) {
//				UpdateLEDs("i:ss");
//			} else if (arm.getInfraredSensor() == false) {
//				UpdateLEDs("i:sf");
//			}
//		} else if (intake.isRunning() == false) {
//			if (arm.getInfraredSensor() == true) {
//				UpdateLEDs("i:fs");
//			} else if (arm.getInfraredSensor() == false) {
//				UpdateLEDs("i:ff");
//			}
//		}
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
		// SmartDashboard.putNumber("Talon Enc Distance",
		// drive.getTalonDistanceLeft());
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
		// SmartDashboard.putNumber("PID error",
		// drive.motors[2].getClosedLoopError(0));
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
		// SmartDashboard.putBoolean("Top Carriage Hal",
		// !lift.topCarriageHal.get());
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
		// SmartDashboard.putNumber("Current 11",
		// PDPJNI.getPDPChannelCurrent(11, 0));
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

//	public void logNewEvent(String eventToLog) {
//		if (useLogging == true) {
//			logger.info(returnDifferenceInMatchTime() + " " + eventToLog);
//		} else if (useLogging == false) {
//			logger.info("turned off for match");
//		}
//	}
//
//	public String returnDifferenceInMatchTime() {
//		if (DriverStation.getInstance().isAutonomous()) {
//			modeTime = autoTime;
//			currentTime = DriverStation.getInstance().getMatchTime();
//		}
//		if (DriverStation.getInstance().isOperatorControl()) {
//			modeTime = teleOpTime;
//			currentTime = DriverStation.getInstance().getMatchTime();
//		}
//		conversionTime = Double.toString(modeTime - currentTime);
//		return conversionTime;
//	}

	public static void UpdateLEDs(String WriteString) {

		char[] CharArray = WriteString.toCharArray();
		byte[] RobotStatus = new byte[CharArray.length];
		for (int i = 0; i < CharArray.length; i++) {
			RobotStatus[i] = (byte) CharArray[i];
		}
		arduino.writeBulk(RobotStatus, RobotStatus.length);
	}
}
