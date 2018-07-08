
package org.usfirst.frc.team649.robot;

import java.util.concurrent.ScheduledExecutorService;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.autonomous.AutoTest;
import org.usfirst.frc.team649.autonomous.CenterSwitchRight;
import org.usfirst.frc.team649.autonomous.DriveStraight;
import org.usfirst.frc.team649.autonomous.LeftFarScale;
import org.usfirst.frc.team649.autonomous.LeftScaleMP;
import org.usfirst.frc.team649.autonomous.LeftSwitch;
import org.usfirst.frc.team649.autonomous.MiddleRightDouble;
import org.usfirst.frc.team649.autonomous.RightFarScale;
import org.usfirst.frc.team649.autonomous.RightScale;
import org.usfirst.frc.team649.autonomous.RightScaleNoTurn;
import org.usfirst.frc.team649.autonomous.RightSwitch;
import org.usfirst.frc.team649.robot.CommandGroups.CenterSwitchRightDoubleMP;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossible2ndIntakeFront;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossible2ndIntakeRear;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeFront;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeRear;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleStoreFront;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleStoreRear;
import org.usfirst.frc.team649.robot.CommandGroups.LeftMPSwitch;
import org.usfirst.frc.team649.robot.CommandGroups.LeftScaleDoubleScaleMP;
import org.usfirst.frc.team649.robot.CommandGroups.RightMPSwitch;
import org.usfirst.frc.team649.robot.CommandGroups.RightScaleDoubleScaleMP;

import org.usfirst.frc.team649.robot.CommandGroups.RightScaleSingleMP;
import org.usfirst.frc.team649.robot.commands.Diagnostic;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;

import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.MoveArmCommand;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.intake.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.liftCommands.MoveLiftCommand;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.AutoSelector;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroBackSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem.LiftStateConstants;
import org.usfirst.frc.team649.robot.util.CameraServer;
import org.usfirst.frc.team649.robot.util.Lidar;
import org.usfirst.frc.team649.robot.util.VoltageLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends TimedRobot {

	public static OI oi;

	// *************************************************** Subsystems
	public static DrivetrainSubsystem drive;
	public static GyroSubsystem gyro;
	public static Lidar lidar;
	public static ArmSubsystem arm;
	public static LiftSubsystem lift;
	public static IntakeSubsystem intake;

	public static Compressor compressor;
	public static AutoTest autoTest;
	public static AutoSelector switches;

	public static boolean canFlipArm;
	public static boolean isPIDActive;
	public static boolean isAutoShift;
	public static boolean isVPid;
	public static boolean isHigh;
	public static boolean isTuningPID = true;
	public static boolean isZero;
	public static GyroBackSubsystem gyroBack;
	public static boolean isArmPidRunning;
	public static boolean isDrivePIDRunning;
	public static boolean isLiftPidRunning;
	public static boolean isLiftStall;
	public static boolean armIsFront;
	public static boolean isTestingAuto = true;
	public static boolean isOpen;
	public static boolean isMPRunning;

	// *********************************************************** Prev State
	// Booleans
	public static boolean liftManualPrevState;
	public static boolean armManualPrevState;
	public static boolean prevStateFlipArm;
	public static boolean prevStateIntakeToggle;
	public static boolean prevStateIntakeToggle2;
	public static boolean prevStateFlipAndStore;
	public static boolean prevStateFlipAndIntakeHigh;
	public static boolean prevStateFlipAndIntakeLow;

	public static int tuningConstant;
	public static double lidarValue;

	public double lidarCount;

	// ********************************************************* Timers
	public Timer time;
	public Timer matchTimer;
	public Timer intakeTimer;
	public double teleOpTime = 135;
	public double autoTime = 15;
	public double currentTime;
	public String conversionTime;
	public double modeTime;

	public static int timeoutMs = 20;

	public static double robotLength = 32; 
	public double lidarOffset;

	public static int liftState;
	public static int liftHalState;
	public static int customLiftPos;
	int timesCalled;
	public static int armState;
	public static int customArmPos;

	public static int program = 1;

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

	public static Trajectory.Config configSideBack;
	public static Trajectory trajectorySideBack;
	public static TankModifier modifierSideBack;

	public static boolean autoShiftButtonPrevState;
	public static boolean VPidButtonPrevState;

	public static SerialPort sp;
	int state;

	public static VoltageLog log;
	public static PowerDistributionPanel pdp;

	public static I2C arduino;
	public boolean hasEndgameStarted;

	public static boolean enteredManualMode;
	public DriverStation.Alliance alliance;
	public static boolean isSerialInitiated;
	public static boolean hasFMS;
	public static boolean shouldSwitchTurnRatio;
	public static boolean isAutoInTeleopPrev;
	public static boolean isIntakeOpen;
	public static boolean shouldCanclArmMP;
	public static boolean isRunnigWithFlip;
	public static boolean endAuto;
	public static Timer auto;

	public static int pos = 4; // left mid right forward

	@Override

	public void robotInit() {
		oi = new OI();
		compressor = new Compressor(4);
		endAuto = false;
		auto = new Timer();
		// pdp = new PowerDistributionPanel(RobotMap.POWER_DISTRIBUTION_PANEL);
		// log = new VoltageLog(pdp,compressor);
		lift = new LiftSubsystem();
		drive = new DrivetrainSubsystem();
		gyro = new GyroSubsystem();
		arm = new ArmSubsystem();
		intake = new IntakeSubsystem();
		lidarCount = 0;
		shouldCanclArmMP = false;
		isRunnigWithFlip = false;
		gyroBack = new GyroBackSubsystem();

		switches = new AutoSelector();
		intakeTimer = new Timer();

		isHigh = false;
		lidar = new Lidar(I2C.Port.kOnboard, 0xC4 >> 1);
		isPIDActive = false;
		tuningConstant = 1;
		isZero = false;
		time = new Timer();
		canFlipArm = false;
		liftState = 2;
		alliance = DriverStation.getInstance().getAlliance();
		if (arm.getArmRaw() < ArmSubsystem.ArmEncoderConstants.INTAKE_REAR / 2) {
			armState = ArmSubsystem.ArmStateConstants.INTAKE_REAR;
		} else {
			armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
		}
		timesCalled = 0;
		lidarOffset = 0;
		lidarValue = lidar.getSample();

		isOpen = false;
		isMPRunning = false;
		isIntakeOpen = false;
		prevStateIntakeToggle = false;
		prevStateIntakeToggle2 = false;
		prevStateFlipAndStore = false;
		prevStateFlipAndIntakeHigh = false;
		prevStateFlipAndIntakeLow = false;
		new Thread(() -> {
			// 10.6.49.7 = tanAxisCamera
			AxisCamera camera = CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axisName,
					RobotMap.Camera.axisPort);
			camera.setResolution(RobotMap.Camera.axisResWidth, RobotMap.Camera.axisResWidth);
			camera.setFPS(RobotMap.Camera.axisFPS);
			CvSink cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axisName);
			CvSource outputStream = CameraServer.getInstance().putVideo("649Camera", 240, 360);//320 480

			Mat frame = new Mat();

			while (!Thread.interrupted()) {

				if (cvSink.grabFrame(frame) == 0) {
					continue;
				}
				outputStream.putFrame(frame);

			}
		}).start();
		isAutoInTeleopPrev = false;
		if (pos == 0) {
			Waypoint[] pointsLeftScaleSingle = new Waypoint[] { new Waypoint(-12.9, 3.1, 0),
					new Waypoint(-6, 3.0, Pathfinder.d2r(-15)),
					new Waypoint(0.8, 1, Pathfinder.d2r(-60)) };

			configLeftScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 5.5, 2.6, 7); //3.1 accel
			trajectoryLeftScaleSingle = Pathfinder.generate(pointsLeftScaleSingle, configLeftScaleSingle);
			modifierLeftScaleSingle = new TankModifier(trajectoryLeftScaleSingle).modify(0.66);

		} else if (pos == 1) {
			Waypoint[] pointsMiddleRightSingle = new Waypoint[] { new Waypoint(-5.8, 0, 0), new Waypoint(0, 0, 0) };

			configMiddleRightSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
			trajectoryMiddleRightSingle = Pathfinder.generate(pointsMiddleRightSingle, configMiddleRightSingle);
			modifierMiddleRightSingle = new TankModifier(trajectoryMiddleRightSingle).modify(0.66);
			Waypoint[] pointsMiddleLeftSingle = new Waypoint[] { new Waypoint(-1.2, -5, 0),
					new Waypoint(-.7, -5, Pathfinder.d2r(30)), new Waypoint(0, 0, 0), };
			configMiddleLeftSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 3, 2.3, 12);
			trajectoryMiddleLeftSingle = Pathfinder.generate(pointsMiddleLeftSingle, configMiddleLeftSingle);
			modifierMiddleLeftSingle = new TankModifier(trajectoryMiddleLeftSingle).modify(0.66);

		} else if (pos == 2) {
			Waypoint[] pointsRightScaleSingle = new Waypoint[] { new Waypoint(-12.9, -3.1, 0),
					new Waypoint(-6, -3.0, Pathfinder.d2r(15)),
					new Waypoint(0.8, -0.1, Pathfinder.d2r(45)) };
				
			configRightScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
			Trajectory.Config.SAMPLES_HIGH, 0.02, 7.5, 3.1, 12);
			trajectoryRightScaleSingle = Pathfinder.generate(pointsRightScaleSingle, configRightScaleSingle);
			modifierRightScaleSingle = new TankModifier(trajectoryRightScaleSingle).modify(0.66);
			
		}

	}
	
	public void leftScaleMP() {
		Waypoint[] pointsLeftScaleSingle = new Waypoint[] { new Waypoint(-12.9, 3.1, 0),
				new Waypoint(-6, 3.0, Pathfinder.d2r(-15)),
				new Waypoint(0.8, 1, Pathfinder.d2r(-60)) };

		configLeftScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.02, 7.5, 3.1, 12);
		trajectoryLeftScaleSingle = Pathfinder.generate(pointsLeftScaleSingle, configLeftScaleSingle);
		modifierLeftScaleSingle = new TankModifier(trajectoryLeftScaleSingle).modify(0.66);
		Waypoint[] sideBack = new Waypoint[] { new Waypoint(0, 0, Pathfinder.d2r(0)), new Waypoint(0.1, 0, Pathfinder.d2r(0)) };
		configSideBack = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
				0.02, 1, 1.5, 3);
		trajectorySideBack = Pathfinder.generate(sideBack, configSideBack);
		modifierSideBack = new TankModifier(trajectorySideBack).modify(0.66);
	}

	@Override
	public void disabledInit() {
		// log.endLogging();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();

	}

	@Override
	public void autonomousInit() {
		// TODO Comment back in: log.startLoggingWithInterval("practicing",
		// 100L);
		auto.start();
		hasFMS = false;
		compressor.stop();
		shouldSwitchTurnRatio = false;
		for (int i = 0; i < 4; i++) {
			drive.motors[i].setNeutralMode(NeutralMode.Brake);
			drive.motors[i].configMotionAcceleration(9000, timeoutMs);
			drive.motors[i].configMotionCruiseVelocity(18000, timeoutMs);
			drive.motors[i].config_kP(0, 1.5, Robot.timeoutMs);
			drive.motors[i].config_kI(0, 0, Robot.timeoutMs);
			drive.motors[i].config_kD(0, 0.9, Robot.timeoutMs);
		}

		isZero = false;

		drive.resetEncoders();
		gyro.resetGyro();
		drive.shift(true);
		drive.changeBrakeCoast(true);

		new ZeroArmRoutine().start();
//		new RightSwitch().start();
//		new RightFarScale().start();
//		new LeftFarScale().start();
//		new GyroPID(90).start();
//		new LeftSwitch().start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		if (lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE) { // ******** Zero Lift
			lift.resetLiftEncoder();
		}
//		SmartDashboard.putNumber("match timer per part of match", (int) DriverStation.getInstance().getMatchTime());
		 if (arm.getArmHalZeroFront()) {
			 arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, 0, 20);
		 } else if (arm.getArmHalZeroBack()) {
			 arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, 0, 20);
		 }
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gameData.length() > 0 && !hasFMS) {
			hasFMS = true;
			if (pos == 0) { // left
				if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					
//					new LeftFarScale().start();
//					new LeftSwitch().start();
					new DriveStraight().start();

				} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					//new LeftScaleDoubleScaleMP().start();
//					 new LeftScaleMP().start();
//					 new LeftScaleSingleMP().start();
					// new LeftSwitchAround().start();
					// new LeftScale().start();
					new DriveStraight().start();

				} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					// new LeftScaleDoubleScaleMP().start();
//					new LeftScaleMP().start();	
//					 new LeftScaleSingleMP().start();
					// new LeftScaleSWSCMP().start();
//					 new LeftSwitch().start();
					// new LeftScale().start();
					new DriveStraight().start();


				} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					

//					 new LeftFarScale().start();
					// new LeftSwitchAround().start();
					new DriveStraight().start();
				}
			} else if (pos == 1) { // mid
				if (gameData.charAt(0) == 'L') {
					shouldSwitchTurnRatio = true;
					left = new EncoderFollower(modifierMiddleLeftSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierMiddleLeftSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 3, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 3, 0);
					new LeftMPSwitch().start();
//					new RightMPSwitch().start();

					// new CenterSwitchLeft().start();
				} else if (gameData.charAt(0) == 'R') {
					left = new EncoderFollower(modifierMiddleRightSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierMiddleRightSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					new RightMPSwitch().start();
//					new CenterSwitchRight().start();
					
//					left = new EncoderFollower(modifierRightScaleSingle.getLeftTrajectory());
//					right = new EncoderFollower(modifierRightScaleSingle.getRightTrajectory());
//					left.configureEncoder(0, 4096 * 2, 0.127);
//					right.configureEncoder(0, 4096 * 2, 0.127);
//					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
//					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					//new CenterSwitchRightDoubleMP().start();
				}

			} else if (pos == 2) { // right
				if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') { // near switch, far scale
					new RightSwitch().start();
				} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') { // far switch, near scale
					left = new EncoderFollower(modifierRightScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierRightScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
//					new RightScaleSingleMP().start();
					new DriveStraight().start();
				} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') { // near switch, near scale
					//TODO: Change command and encoder values if we want to run switch instead
					left = new EncoderFollower(modifierRightScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierRightScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
//					new RightScaleSingleMP().start();
//					new DriveStraight().start();
					new RightSwitch().start();

				} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') { // far switch, far scale
					//TODO: Set the encoder values if we decide to make this MP
//					new RightFarScale().start();
					new DriveStraight().start();

				}
			}
		}
		updateSmartDashboardTesting();
	}

	@SuppressWarnings("unused")
	@Override
	public void teleopInit() {
		auto.reset();

		compressor.start();

		// log.startLoggingWithInterval("practicing", 100L);

		intakeTimer.start();
		isZero = true;
		drive.changeBrakeCoast(false);

		isAutoShift = true;
		isVPid = true;
		endAuto = true;
		isArmPidRunning = false;
		
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();

		drive.resetEncoders();
		time.start();

		isDrivePIDRunning = false;
		isArmPidRunning = false;
		isLiftPidRunning = false;
		isLiftStall = false;

		customLiftPos = (int) lift.getRawLift();
		customArmPos = (int) arm.getArmRaw();

		new SetIntakePistons(false, true).start();
		isOpen = false;
		drive.shift(false);

		SmartDashboard.putBoolean("Going Back to front Op 8 Left", false);
		SmartDashboard.putBoolean("Flipping Arm Op 6", false);
		SmartDashboard.putBoolean("In Intake Low Flip", false);
		SmartDashboard.putBoolean("Move Arm to Back", false);

		// ***************************************************** Prev States
		autoShiftButtonPrevState = false;
		VPidButtonPrevState = false;

		liftManualPrevState = false;
		armManualPrevState = false;
		prevStateFlipArm = false;

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		// teleopRun();
		cleanTeleopRun();
		updateSmartDashboardTesting();
	}

	// public void teleopRun() {
	// // if (oi.driver.shiftUp()) {
	// // drive.shift(true);
	// // } else {
	// // drive.shift(false);
	// // }
	// if (oi.operator.turnOffCompressorManually())
	// {
	// compressor.stop();
	// }
	// if (oi.operator.turnOnCompressorManually())
	// {
	// compressor.start();
	// }
	// }

	public void cleanTeleopRun() {
		// *********************************************************************************** driving and shifting
		if (oi.operator.newShiftToggle()) {
			isHigh = !isHigh;
			drive.shift(isHigh);
		}
		drive.driveForwardRotateTeleop(oi.driver.getForward(), oi.driver.getRotation());


		if (oi.operator.getIntakeState()) { // *********************************** Move Lift to bottom, arm to intake
			new MoveLiftCommand(LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.INTAKE_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.INTAKE_REAR, false).start();
			}
		} else if (oi.operator.getStoreState()) { // **************************** Move Lift to bottom, arm to store
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.STORE_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.STORE_REAR, false).start();
			}
		} else if (oi.operator.getScaleLowState()) { // ************************* Move Lift to low Scale, arm to mid drop
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.MID_DROP_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.MID_DROP_REAR, false).start();
			}
		} else if (oi.operator.getScaleLowMidState()) { // ************************* Move Lift to low Scale, arm to mid drop
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.LOW_MID_SCALE, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.MID_DROP_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.MID_DROP_REAR, false).start();
			}
			
		} else if (oi.operator.getScaleMidState()) { // ************************* Move Lift to mid scale, arm to mid drop
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.MID_SCALE_STATE, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.MID_DROP_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.MID_DROP_REAR, false).start();
			}
		} else if (oi.operator.getScaleHighState()) { // ************************ Move Lift to high scale, arm to high drop
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR, false).start();
			}
		} else if (oi.operator.getExchangeState()) { // ************************* Move Lift to high intake, arm to exchange
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.INTAKE_2, false).start();
			if (armIsFront) {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT, false).start();
			} else {
				new MoveArmCommand(ArmSubsystem.ArmStateConstants.EXCHANGE_REAR, false).start();
			}
		} else if (oi.operator.getArmUpSmall()) { // ****************************** Move arm by small degree towards back
			new MoveArmCommand(ArmSubsystem.ArmStateConstants.CUSTOM, true).start();
		} else if (oi.operator.getArmDownSmall()) { // **************************** Move arm by small degree towards front
			new MoveArmCommand(ArmSubsystem.ArmStateConstants.CUSTOM, false).start();
		} else if (oi.operator.getLiftUpSmall()) { // ***************************** Move lift by small degree upwards
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.CUSTOM_STATE, true).start();
		} else if (oi.operator.getLiftDownSmall()) { // *************************** Move lift by small degree downwards
			new MoveLiftCommand(LiftSubsystem.LiftStateConstants.CUSTOM_STATE, false).start();
		} else if (oi.operator.flipAndIntakeLow()) {
			if (isOpen) {
				new SetIntakePistons(false, true).start();
			}
			if (lift.isCarriageAtBottom()) {
				if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
						&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
					liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
					new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
				}
				if (armIsFront) {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState, false).start();
					}

				} else {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState, false).start();
					}
				}
			} else {
				if (!isRunnigWithFlip) {
					isRunnigWithFlip = true;
					if (armIsFront) {
						new DownAndFlipWhenPossibleIntakeRear().start();
					} else {
						new DownAndFlipWhenPossibleIntakeFront().start();
					}
				}
			}

		} else if (oi.operator.flipAndIntakeHigh()) {
			if (isOpen) {
				new SetIntakePistons(false, true).start();
			}
			if (lift.isCarriageAtBottom()) {
				if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2
						&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_2) {
					liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2;
					new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE, liftState, 0).start();
				}
				if (armIsFront) {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;

						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState, false).start();
					}

				} else {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState, false).start();
					}

				}
			} else {
				if (!isRunnigWithFlip) {
					isRunnigWithFlip = true;

					if (armIsFront) {
						new DownAndFlipWhenPossible2ndIntakeRear().start();
					} else {
						new DownAndFlipWhenPossible2ndIntakeFront().start();
					}
				}

			}
		} else if (oi.operator.flipAndStore()) {
			if (isOpen) {
				new SetIntakePistons(false, true).start();
			}
			if (lift.isCarriageAtBottom()) {
				if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
						&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
					liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
					new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
				}
				if (armIsFront) {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState, false).start();
					}

				} else {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState, false).start();
					}

				}
			} else {
				if (!isRunnigWithFlip) {
					isRunnigWithFlip = true;
					if (armIsFront) {
						new DownAndFlipWhenPossibleStoreRear().start();
					} else {
						new DownAndFlipWhenPossibleStoreFront().start();
					}
				}
			}
		} else if (oi.operator.isManual()) { // ************************************************ Manual Mode
			SmartDashboard.putBoolean("Is stalling", false);
			if (enteredManualMode == false) {
				enteredManualMode = true;
			}
			SmartDashboard.putBoolean("Manual", enteredManualMode);
			// ********************************************************************************* Manual setup
			liftState = LiftSubsystem.LiftStateConstants.CUSTOM_STATE;
			armState = ArmSubsystem.ArmStateConstants.CUSTOM;

			if (lift.isSecondStageAtBottom() && lift.isCarriageAtBottom()) { // **************** if lift at bottom only run up
				if (oi.operator.getOperatorY() < 0) {
					lift.setLift(0);
				} else {
					lift.setLift(oi.operator.getOperatorY());
				}
			} else {
				lift.setLift(oi.operator.getOperatorY());
			}
			// ******************************************************************************** Update Lift Positions
			customLiftPos = (int) lift.getRawLift();
			customArmPos = (int) arm.getArmRaw();

			double armJoy = oi.operator.getManualArm(); // ************************************ Manual Arm Code
			if (armJoy == 0) {
				arm.setArm(armJoy);
				if (time.get() > 0.3) {
					arm.setArmBrake(true);
				}
			} else {
				arm.setArm(armJoy / 3);
				arm.setArmBrake(false);
				time.reset();
			}
		} else { // ****************************************************************************** Stall motors for lift
			SmartDashboard.putBoolean("Is stalling", true);
			SmartDashboard.putBoolean("Manual", false);
			if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				lift.setLift(0);
			} else if (liftState == LiftSubsystem.LiftStateConstants.SWITCH_STATE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.SWITCH_STATE);
			} else if (liftState == LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE);
			} else if (liftState == LiftSubsystem.LiftStateConstants.LOW_MID_SCALE) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.LOW_MID_SCALE);
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
			} else if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_2) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE);
			}
			if (!isArmPidRunning) { // ************************************************************* Turn on arm brake
				arm.setArmBrake(true);
			}
			if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT) { // ********************** Zero Arm front
				arm.setArm(0);
			} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) { // ***************** Zero Arm Back
				arm.setArm(0);
			}
			if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				compressor.start();
			}else{
				compressor.stop();
			}

		}

		if((armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT ||armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) && !Robot.arm.getInfraredSensor() && oi.operator.deployOnlyWheels()){ //Robot.arm.getInfraredSensor()
			new SetIntakePistons(false, true).start();
			new RunIntakeWheels(-0.85).start();

		}else if((armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT ||armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) && !Robot.arm.getInfraredSensor() && oi.operator.lowSpeedDeploy()){ //Robot.arm.getInfraredSensor()
			new SetIntakePistons(false, true).start();
			new RunIntakeWheels(-0.35).start();
			
		}
		else if((armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT ||armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) && Robot.arm.getInfraredSensor() && oi.operator.runIntakeWithWheelsClosed()){
			new SetIntakePistons(false, true).start();
			new RunIntakeWheels(1).start();
		}else if((armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT ||armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) && Robot.arm.getInfraredSensor()){
			new SetIntakePistons(false, true).start();
			new RunIntakeWheels(0).start();

		}else if (oi.operator.deployOnlyWheels()) { // **************************************************** Deploy intake wheels
			new RunIntakeWheels(-1).start();
		} else if (oi.operator.lowSpeedDeploy()) { // *********************************************** slow intake deploy
			new RunIntakeWheels(-0.35).start();
		} else if (oi.operator.openIntakeToggle() && oi.operator.runIntakeWithWheelsClosed()) { // ** Open intakes and run wheels
			new SetIntakePistons(true, false).start();
			new RunIntakeWheels(1).start();
		} else if (oi.operator.openIntakeToggle()) { // ********************************************* Open Intakes
			new SetIntakePistons(true, false).start();
			new RunIntakeWheels(0).start();
		} else if (oi.operator.runIntakeWithWheelsClosed()) { // ************************************ Close intakes and run wheels
//			if (!arm.getInfraredSensor()) {
			if (arm.getInfraredSensor()) {
				new SetIntakePistons(false, false).start();
			}
			new RunIntakeWheels(1).start();
		} else if (isOpen == false && !oi.operator.runIntakeWithWheelsClosed()
				&& !(oi.operator.openIntakeToggle() || oi.operator.openIntakeToggleBB())) { // ****** open intakes
			new SetIntakePistons(false, true).start();
			Robot.intake.setIntakeMotors(0, 0);
		} else { // ********************************************************************************* close and clamp
			new SetIntakePistons(false, true).start();
			Robot.intake.setIntakeMotors(0, 0);
		}
		if (arm.getArmRaw() > (ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT
				+ ArmSubsystem.ArmEncoderConstants.INTAKE_REAR) / 2) {
			armIsFront = true;
		} else {
			armIsFront = false;

		}
		if (lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE) { // ******** Zero Lift
			lift.resetLiftEncoder();
		}
		SmartDashboard.putNumber("match timer per part of match", (int) DriverStation.getInstance().getMatchTime());
		 if (arm.getArmHalZeroFront()) {
			 arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, 0, 20);
		 } else if (arm.getArmHalZeroBack()) {
//			 arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, 0, 20);
		 }
		if (arm.getArmHalZeroFront() || oi.operatorJoystick.getRawButton(7)) { // ********************* Zero Arm Front
			arm.setEncoder(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT);
		} else if (arm.getArmHalZeroBack()) {
			arm.setEncoder(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR);
		}
		if (lift.isSecondStageAtBottom()) { // ******************************************* Take lidar values
			lidarOffset = lidar.getSample();
		}
		if (lidarCount == 10) { // ********************************************************* lidar sample rate
			lidarValue = lidar.getSample() - lidarOffset;
			lidarCount = 0;
		}
		lidarCount++;
		SmartDashboard.putNumber("adj Lidar", lidarValue);
		SmartDashboard.putBoolean("Can Flip", lift.canFlip());
		SmartDashboard.putBoolean("ir", arm.getInfraredSensor());
		SmartDashboard.putNumber("Arm Raw", arm.getArmRaw());

		// ***************************************************************** Prev States
		prevStateIntakeToggle = oi.operator.openIntakeToggleBB();
		prevStateIntakeToggle2 = oi.operator.openIntakeToggle();
		prevStateFlipAndIntakeHigh = oi.operator.flipAndIntakeHigh();
		prevStateFlipAndIntakeLow = oi.operator.flipAndIntakeLow();
		prevStateFlipAndStore = oi.operator.flipAndStore();
		if (oi.operatorJoystick.getRawButton(10)) {
			lift.mainLiftMotor.setSelectedSensorPosition(0, 0, timeoutMs);
		} else if (oi.operatorJoystick.getRawButton(1)) {
			lift.mainLiftMotor.setSelectedSensorPosition(-1000, 0, timeoutMs);
		}
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
		SmartDashboard.putNumber("ARM ENCODER", arm.bottomMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Talon Enc Distance Left", drive.motors[0].getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Talon Enc Distance Right", drive.motors[2].getSelectedSensorPosition(0));
		SmartDashboard.putBoolean("is Second Stage at Bottom", lift.isSecondStageAtBottom());
		SmartDashboard.putBoolean("is Second Stage at Top", lift.isSecondStageAtTop());
		SmartDashboard.putBoolean("is Carriage at Bottom", lift.isCarriageAtBottom());
		SmartDashboard.putBoolean("is Carriage at Top", lift.isCarriageAtTop());
		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
		SmartDashboard.putNumber("Arm Raw", arm.getArmRaw());

		if (lidarCount == 12) {
			SmartDashboard.putNumber("Lidar", lidar.getSample());
			lidarCount = 0;
		}
		lidarCount++;

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

}
