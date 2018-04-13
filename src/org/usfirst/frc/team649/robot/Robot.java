
package org.usfirst.frc.team649.robot;
import java.util.concurrent.ScheduledExecutorService;
import org.opencv.core.Mat;
import org.usfirst.frc.team649.autonomous.AutoTest;
import org.usfirst.frc.team649.autonomous.DriveStraight;
import org.usfirst.frc.team649.autonomous.LeftSwitch;
import org.usfirst.frc.team649.autonomous.RightSwitch;
import org.usfirst.frc.team649.autonomous.autoMaster;
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
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.AutoSelector;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.robot.util.CameraServer;
import org.usfirst.frc.team649.robot.util.Lidar;
import org.usfirst.frc.team649.robot.util.RunnableLEDs;
import org.usfirst.frc.team649.robot.util.VoltageLog;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
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
	public static Lidar lidar;
	public static ArmSubsystem arm;
	public static LiftSubsystem lift;
	public static Compressor compressor;
	public static autoMaster automaster;
	public static AutoTest autoTest;
	public static AutoSelector switches;
	public static boolean drivePIDRunning;
	public static double lidarValue;
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

	public static Trajectory.Config configSideBack;
	public static Trajectory trajectorySideBack;
	public static TankModifier modifierSideBack;
	// prev state variables leave at bottom

	// these two are for buttons not the actual
	public static boolean autoShiftButtonPrevState;
	public static boolean VPidButtonPrevState;

	public static boolean liftManualPrevState;
	public static boolean armManualPrevState;
	public static boolean prevStateFlipArm;
	public static boolean prevStateIntakeToggle;
	public static boolean prevStateIntakeToggle2;
	public static boolean prevStateFlipAndStore;
	public static boolean prevStateFlipAndIntakeHigh;
	public static boolean prevStateFlipAndIntakeLow;

	public static SerialPort sp;
	int state;
	public static int ledState;
	private ScheduledExecutorService leds;
	private Runnable rLEDs;
	public static VoltageLog log;
	public static PowerDistributionPanel pdp;
	
	public static boolean enteredManualMode;
	public boolean hasEndgameStarted;
	public DriverStation.Alliance alliance;
	public static I2C arduino;
	public static boolean isSerialInitiated;
	public static boolean hasFMS;
	public static boolean shouldSwitchTurnRatio;
	public static boolean isAutoInTeleopPrev;
	public static boolean isIntakeOpen;
	public static boolean shouldCanclArmMP;
	public static boolean isRunnigWithFlip;
	public static int pos = 2; // left mid right forward

	@Override
    
	public void robotInit() {
		compressor = new Compressor(4);
		pdp = new PowerDistributionPanel(RobotMap.POWER_DISTRIBUTION_PANEL);
		log = new VoltageLog(pdp,compressor);
		lift = new LiftSubsystem();
		drive = new DrivetrainSubsystem();
		gyro = new GyroSubsystem();
		arm = new ArmSubsystem();
		intake = new IntakeSubsystem();
		
		oi = new OI();

//		rLEDs = new RunnableLEDs();
		
		drivePIDRunning = false;
		
		lidarCount = 0;
		shouldCanclArmMP = false;
		isRunnigWithFlip = false;
		
		// switches = new AutoSelector();
		intakeTimer = new Timer();
		
		isHigh = false;
		// automaster = new autoMaster();
		lidar = new Lidar(I2C.Port.kOnboard, 0xC4 >> 1);
		isPIDActive = false;
		tuningConstant = 1;
		isZero = false;
		time = new Timer();
		canFlipArm = false;
		prevDriveVel = 0;
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
//		rLEDs = new RunnableLEDs();
//		sp = new SerialPort(115200, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
//		leds = Executors.newSingleThreadScheduledExecutor();
//		leds.scheduleWithFixedDelay(rLEDs, 100L, 100L, TimeUnit.MILLISECONDS);
		new Thread(() -> {
			// 10.6.49.7 = tanAxisCamera
			AxisCamera camera = CameraServer.getInstance().addAxisCamera(RobotMap.Camera.axisName,
					RobotMap.Camera.axisPort);
			camera.setResolution(RobotMap.Camera.axisResWidth, RobotMap.Camera.axisResWidth);
			camera.setFPS(RobotMap.Camera.axisFPS);
			CvSink cvSink = CameraServer.getInstance().getVideo(RobotMap.Camera.axisName);
			CvSource outputStream = CameraServer.getInstance().putVideo("649Camera", 320, 480);

			Mat frame = new Mat();

			while (!Thread.interrupted()) {

				if (cvSink.grabFrame(frame) == 0) {
					continue;
				}
				outputStream.putFrame(frame);

			}
		}).start();
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
		// TankModifier(t rajectoryLeftScaleSingle).modify(0.66);
		isAutoInTeleopPrev = false;
		//
		// File leftScaleSingle = new
		// File("C:\\Users\\MSET\\workspace\\leftScaleSingle.csv");
		// Pathfinder.writeToCSV(leftScaleSingle,
		// modifierLeftScaleSingle.getLeftTrajectory());
		//
		if (alliance == DriverStation.Alliance.Blue) {
//			setLEDs(1);// blue
		} else if (alliance == DriverStation.Alliance.Red) {
//			setLEDs(2);// red
		} else {
//			setLEDs(12);// yellow
		}

		// Waypoint[] pointsRightScaleSingle = new Waypoint[] {
		// new Waypoint(-12.6,-2.9,0),
		// new Waypoint(-7,-2.9,0),
		// new Waypoint(-4.3,-2.9,Pathfinder.d2r(45)),
		// new Waypoint(-2.05,0,0),
		// new Waypoint(0,0,0)
		// };
		//
		// configRightScaleSingle = new
		// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUB
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
		ledState = 0;

		if (pos == 0) {
			Waypoint[] pointsLeftScaleSingle = new Waypoint[] { new Waypoint(-12.9, 3.1, 0), new Waypoint(-7, 3.1, 0),
					new Waypoint(-4, 3.1, Pathfinder.d2r(-45)), new Waypoint(-2.05, 0, 0), new Waypoint(0, 0, 0) };

			configLeftScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 4, 2.9, 12);
			trajectoryLeftScaleSingle = Pathfinder.generate(pointsLeftScaleSingle, configLeftScaleSingle);
			modifierLeftScaleSingle = new TankModifier(trajectoryLeftScaleSingle).modify(0.66);
			
			Waypoint[] sideBack = new Waypoint[] {
					new Waypoint(-2,0,0),
					new Waypoint(0,0,0)
			};
			configSideBack = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 4, 2.9, 12);
			trajectorySideBack = Pathfinder.generate(sideBack, configSideBack);
			modifierSideBack = new TankModifier(trajectorySideBack).modify(0.66);
			
		} else if (pos == 1) {
			Waypoint[] pointsMiddleRightSingle = new Waypoint[] { new Waypoint(-5.8, 0, 0), new Waypoint(0, 0, 0) };

			configMiddleRightSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 3.3, 12);
			trajectoryMiddleRightSingle = Pathfinder.generate(pointsMiddleRightSingle, configMiddleRightSingle);
			modifierMiddleRightSingle = new TankModifier(trajectoryMiddleRightSingle).modify(0.66);
			Waypoint[] pointsMiddleLeftSingle = new Waypoint[] { new Waypoint(-1.2, -4, 0),
					new Waypoint(-.7, -4, Pathfinder.d2r(30)), new Waypoint(0, 0, 0), };
			configMiddleLeftSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 3, 2.3, 12);
			trajectoryMiddleLeftSingle = Pathfinder.generate(pointsMiddleLeftSingle, configMiddleLeftSingle);
			modifierMiddleLeftSingle = new TankModifier(trajectoryMiddleLeftSingle).modify(0.66);
		} else if (pos == 2) {
			Waypoint[] pointsRightScaleSingle = new Waypoint[] { new Waypoint(-12.9, -3.1, 0),
					new Waypoint(-7, -3.1, 0), new Waypoint(-4, -3.1, Pathfinder.d2r(45)), new Waypoint(-2.5, 0, 0),
					new Waypoint(0, 0, 0) };
			
			configRightScaleSingle = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_HIGH, 0.02, 4, 2.9, 12);
			trajectoryRightScaleSingle = Pathfinder.generate(pointsRightScaleSingle, configRightScaleSingle);
			modifierRightScaleSingle = new TankModifier(trajectoryRightScaleSingle).modify(0.66);
			Waypoint[] sideBack = new Waypoint[] {
					new Waypoint(-2,0,0),
					new Waypoint(0,0,0)
			};
			configSideBack = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 4, 2.9, 12);
			trajectorySideBack = Pathfinder.generate(sideBack, configSideBack);
			modifierSideBack = new TankModifier(trajectorySideBack).modify(0.66);
		}
	}

	@Override
	public void disabledInit() {
		log.endLogging();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();

	}

	@Override
	public void autonomousInit() {
		log.startLoggingWithInterval("practicing", 100L);
		hasFMS = false;
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

		// arm.setArmBrake(true);
		drive.resetEncoders();
		gyro.resetGyro();
		drive.shift(true);
		drive.changeBrakeCoast(true);	
		new ZeroArmRoutine().start();
//		new DriveBackForTime(-0.3,0.9).start();
//		left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
//		right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
//		left.configureEncoder(0, 4096 * 2, 0.127);
//		right.configureEncoder(0, 4096 * 2, 0.127);
//		left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
//		right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
//		new MotionProfileDrive(true).start();
//		 new RighScaleSWSCMP().start();
//		new GyroPID(90).start();

//		new DrivetrainMotionProfileIn(100).start();
		// arm.bottomMotor.set(ControlMode.MotionMagic, 435);
		// new EncoderTurn(90).start();
		// new LeftSwitch().start();
		// new LeftSwitch().start();
		// lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE);
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
//		Robot.drive.motors[0].set(ControlMode.MotionMagic, -200000);
//		Robot.drive.motors[1].set(ControlMode.MotionMagic, 10000);
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0 && !hasFMS) {
			hasFMS = true;
			//
			if (pos == 0) { // left
				if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
//					 new LeftFarScale().start();
					new LeftSwitch().start();
				} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					new LeftScaleDoubleScaleMP().start();
					// new LeftScaleSingleMP().start();
					// new LeftSwitchAround().start();
					// new LeftScale().start();
				} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					left = new EncoderFollower(modifierLeftScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierLeftScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					new LeftScaleDoubleScaleMP().start();
		
					// new LeftScaleSingleMP().start();
//					 new LeftScaleSWSCMP().start();
					// new LeftSwitch().start(); 
					// new LeftScale().start();

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
					// new CenterSwitchLeft().start();
				} else if (gameData.charAt(0) == 'R') {
					left = new EncoderFollower(modifierMiddleRightSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierMiddleRightSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					new RightMPSwitch().start();
					// new CenterSwitchRight().start();
				}

			} else if (pos == 2) { // right
				if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					left = new EncoderFollower(modifierRightScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierRightScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					new RightScaleDoubleScaleMP().start();
					// new RightScaleSingleMP().start();
					// new RightScale().start();
//					new DriveStraight().start();

					// new RightSwitchAround().start();
				} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					// new RightFarScale().start();
					new RightSwitch().start();

				} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
//					new RightFarScale().start();
					new DriveStraight().start();

					// new RightSwitchAround().start();
				} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					left = new EncoderFollower(modifierRightScaleSingle.getLeftTrajectory());
					right = new EncoderFollower(modifierRightScaleSingle.getRightTrajectory());
					left.configureEncoder(0, 4096 * 2, 0.127);
					right.configureEncoder(0, 4096 * 2, 0.127);
					left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
					right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
//					new MotionProfileDrive().start();
					new RightScaleDoubleScaleMP().start();
//					new DriveStraight().start();

					// new RightScaleSingleMP().start();
//					new RighScaleSWSCMP().start();
					// new	f	z	a RightScale().start();
					// new RightSwitch().start();

				}
			} else {
				new DriveStraight().start();
			}
			//
			//
		}

		// SmartDashboard.putNumber("arm", arm.getArmRaw());
		// updateSmartDashboardTesting();
		// SmartDashboard.putNumber("Lift state", liftState);

	}

	@SuppressWarnings("unused")
	@Override
	public void teleopInit() {
		log.startLoggingWithInterval("practicing", 100L);
		armVelMax = 0;
		intakeTimer.start();
		isZero = true;
		// gyro.resetGyro();
		drive.changeBrakeCoast(false);

		// gyro.resetGyro();
		// drive.changeBrakeCoast(false);
		isAutoShift = true;
		maxAccelDrive = 0;
		isVPid = true;
		isArmPidRunning = false;

		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
		// arm.resetEncoder();

		// autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		// VPidButtonPrevState = oi.driver.switchToVbus();
		// arm.resetEncoder();

		drive.resetEncoders();
		// lift.resetLiftEncoder();
		// if(arm.getArmRaw() < ArmSubsystem.ArmEncoderConstants.INTAKE_REAR/2){
		// armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
		// new
		// ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR,armState);
		// }else{
		// armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
		// new
		// ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT,armState);
		//
		// }
		time.start();


		// new SetCompressorCommand(true).start();
//		SmartDashboard.putBoolean("Diag?", false);
		// arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);
		isDrivePIDRunning = false;
		// new Diagnostic().start();
		isArmPidRunning = false;
		isLiftPidRunning = false;
		isLiftStall = false;
		// liftState =
		// LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;

		customLiftPos = (int) lift.getRawLift();

		customArmPos = (int) arm.getArmRaw();
		isOpen = false;
//		new SetIntakePistons(false, true).start();
		drive.shift(false);
		// if (arm.getArmRaw() < ArmSubsystem.ArmEncoderConstants.INTAKE_REAR /
		// 2) {
		// armIsFront = false;
		// armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
		// new
		// ArmMotionProfile(ArmSubsystem.ArmStateConstants.INTAKE_REAR,armState,0)
		// } else {
		// armIsFront = true;
		// armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
		//
		// } // prev state variables leave at bottom

		// these two are for buttons not the actual
		autoShiftButtonPrevState = false;
		VPidButtonPrevState = false;

		liftManualPrevState = false;
		armManualPrevState = false;
		prevStateFlipArm = false;
		// new SetCompressorCommand(true).start();
		// arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);

		//drive.shift(true);
		// new AutoTestCommand().start();
		// drive.shift(true);

		drive.changeBrakeCoast(false);
//		new Diagnostic().start();
		// lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE);
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		teleopRun();
		//drive.shift(false);
		// if (oi.buttonBoard.getRawButton(1)) {
		// SmartDashboard.putBoolean("In DriveTest", true);
		// new DrivetrainMotionProfile(100).start();
		// }else if (oi.buttonBoard.getRawButton(2)) {
		// new DrivetrainMotionProfile(150).start();
		// } else if (oi.buttonBoard.getRawButton(3)) {
		// new DrivetrainMotionProfile(50).start();
		// }
		// SmartDashboard.putBoolean("In DriveTest", false);

	}

	public void teleopRun() {
		// if (oi.driver.shiftUp()) {
		// drive.shift(true);
		// } else {
		// drive.shift(false);
		// }
		if (oi.operator.turnOffCompressorManually())
		{
			compressor.stop();
		}
		if (oi.operator.turnOnCompressorManually())
		{
			compressor.start();
		}
		if (oi.operator.newShiftToggle()) {
			isHigh = !isHigh;
			drive.shift(isHigh);
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
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState,false).start();

				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState,false).start();

				}
			}
		} else if (oi.operator.getStoreState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.STORE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.STORE_REAR) {
				if (armIsFront) {
					timesCalled++;
					SmartDashboard.putNumber("times Call", timesCalled);
					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState,false).start();
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState,false).start();
				}
			}

		}
		// else if (oi.operator.getSwitchState()) {
		// if (liftState !=
		// LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
		// && liftState !=
		// LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
		// liftState =
		// LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
		// new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,
		// liftState, 0).start();
		// }
		// if (armState != ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT
		// && armState != ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR
		// && armState != ArmSubsystem.ArmStateConstants.SWITCH_FRONT
		// && armState != ArmSubsystem.ArmStateConstants.SWITCH_REAR) {
		// if (armIsFront) {
		// armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
		// new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT,
		// armState).start();
		// } else {
		// armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
		// new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR,
		// armState).start();
		// }
		// }
		// }
		else if (oi.operator.getScaleLowState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE, liftState, 0).start();
				SmartDashboard.putBoolean("got in", true);
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState,false).start();
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState,false).start();
				}
			}
		} else if (oi.operator.getScaleMidState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE, liftState, 0).start();
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState,false).start();
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState,false).start();
				}
			}
		} else if (oi.operator.getScaleHighState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE
					&& liftState != LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE, liftState, 0).start();
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState,false).start();
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState,false).start();
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
				new ArmMotionProfile(customArmPos, armState,false).start();
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
				new ArmMotionProfile(customArmPos, armState,false).start();
			}
		} else if (oi.operator.getLiftUpSmall()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP && lift.getRawLift()
					+ LiftSubsystem.LiftEncoderConstants.ADJ_DIST < LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP;
				customLiftPos = (int) lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
				new LiftMotionProfile(customLiftPos, liftState, 0).start();
			}

		} else if (oi.operator.getLiftDownSmall()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN
					&& lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST > 0) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN;
				customLiftPos = (int) lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
				new LiftMotionProfile(customLiftPos, liftState, 0).start();
			}
		} else if (oi.operator.getExchangeState()) {
			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2
					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_2) {
				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE, liftState, 0).start();
			}
			if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR
					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_REAR) {
				if (armIsFront) {
					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState,false).start();
				} else {
					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState,false).start();
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
					// if (armIsFront) {
					if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
							|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState,false)
								.start();
					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
							|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.STORE_FRONT
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;

						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR, armState,false).start();

					}
					// } else {
					if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_REAR
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN
							|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState,false)
								.start();
					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
							|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_REAR
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState,false).start();

					} else if (armState == ArmSubsystem.ArmStateConstants.STORE_REAR
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState,false).start();

					} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;

						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState,false).start();
					} else if (armState == ArmSubsystem.ArmStateConstants.SWITCH_REAR
							|| armState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, armState,false).start();
					}
					// }
				}
			}
		} else if (oi.operator.flipAndIntakeLow()) {
			if(isOpen){
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
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState,false).start();
						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState,false).start();
					}

				} else {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState,false).start();
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
			if(isOpen){
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

						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState,false).start();
					}

				} else {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState,false).start();
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
			if(isOpen){
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
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState,false).start();
					}

				} else {
					if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState,false).start();
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
		} else if (oi.operator.isManual()) {
			if (enteredManualMode == false) {
				enteredManualMode = true;
			}
			liftState = LiftSubsystem.LiftStateConstants.CUSTOM_STATE;
			armState = ArmSubsystem.ArmStateConstants.CUSTOM;
			if (lift.isSecondStageAtBottom() && lift.isCarriageAtBottom()) {
				if (oi.operator.getOperatorY() < 0) {
					lift.setLift(0);
				} else {
					lift.setLift(oi.operator.getOperatorY());
				}
			} else {
				lift.setLift(oi.operator.getOperatorY());
			}
			customLiftPos = (int) lift.getRawLift();
			customArmPos = (int) arm.getArmRaw();

			double armJoy = oi.operator.getManualArm();
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
			} else if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_2) {
				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE);
			}
			if (!isArmPidRunning) {
				arm.setArmBrake(true);
				// if(armState == ArmSubsystem.)
			}
			if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT) {
				arm.setArm(0);
			} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
				arm.setArm(0);
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
		oi.operator.intakePrev();

//		if (oi.operator.deployOnlyWheels()) {
//			// if(armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT||
//			// armState ==
//			// ArmSubsystem.ArmStateConstants.SWITCH_REAR){
//			// new RunIntakeWheels(-0.6);
//			// }else{
//			new RunIntakeWheels(-1).start();
//			// }
//		} else if (oi.operator.lowSpeedDeploy()) {
//			new RunIntakeWheels(-0.55).start();
//		} else if ((oi.operator.openIntakeToggle() && !prevStateIntakeToggle2) || (oi.operator.newBBIntake())) {
//			if (isOpen) {
//				new SetIntakePistons(false, true).start();
//			} else {
//				new SetIntakePistons(true, false).start();
//
//			}
//			Robot.intake.setIntakeMotors(0, 0);
//		} else if (oi.operator.runIntakeWithWheelsClosed()) {
//			new RunIntakeWheels(1).start();
//			if(!isOpen){
//				new SetIntakePistons(false, false).start();
//			}
//
//			// new IntakeWithWheelsAndClose().start();
//		} else if (isOpen == false && !oi.operator.runIntakeWithWheelsClosed()
//				&& !(oi.operator.openIntakeToggle() || oi.operator.openIntakeToggleBB())) {
//			new SetIntakePistons(false, true).start();
//			Robot.intake.setIntakeMotors(0, 0);
//		} else {
//			Robot.intake.setIntakeMotors(0, 0);
//		}
		
		if (oi.operator.deployOnlyWheels()) {
			// if(armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT||
			// armState ==
			// ArmSubsystem.ArmStateConstants.SWITCH_REAR){
			// new RunIntakeWheels(-0.6);
			// }else{
			new RunIntakeWheels(-1).start();
			// }
		} else if (oi.operator.lowSpeedDeploy()) {
			new RunIntakeWheels(-0.35).start();
		} else if (oi.operator.openIntakeToggle() && oi.operator.runIntakeWithWheelsClosed()) {
			new SetIntakePistons(true, false).start();
			new RunIntakeWheels(1).start();
		

		}else if(oi.operator.openIntakeToggle()){
			new SetIntakePistons(true, false).start();
			new RunIntakeWheels(0).start();
		}
		else if (oi.operator.runIntakeWithWheelsClosed()) {
			new SetIntakePistons(false, false).start();

			new RunIntakeWheels(1).start();
//			if(!isOpen){
//				new SetIntakePistons(false, false).start();
//			}

			// new IntakeWithWheelsAndClose().start();
		} else if (isOpen == false && !oi.operator.runIntakeWithWheelsClosed()
				&& !(oi.operator.openIntakeToggle() || oi.operator.openIntakeToggleBB())) {
			new SetIntakePistons(false, true).start();
			Robot.intake.setIntakeMotors(0, 0);
		} else {
			new SetIntakePistons(false, true).start();
			Robot.intake.setIntakeMotors(0, 0);
		}
		if (!arm.getInfraredSensor()) {
//			setLEDs(6);
		}
		if (arm.getInfraredSensor()) {
//			setLEDs(5);
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
		
//		SmartDashboard.putNumber("Gyro", gyro.getGyroAngle());
//		SmartDashboard.putNumber("State", liftState);
//		SmartDashboard.putNumber("arm state", armState);
//		SmartDashboard.putBoolean("isarmfront", armIsFront);
//		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
//		SmartDashboard.putNumber("arm raw", arm.getArmRaw());
//		SmartDashboard.putBoolean("Arm front", arm.getArmHalZeroFront());
//		SmartDashboard.putBoolean("Arm back", arm.getArmHalZeroBack());
//		SmartDashboard.putBoolean("isCarriageAtBot", lift.isCarriageAtBottom());
		// SmartDashboard.putNumber("pot 1", switches.getPot1());
		// SmartDashboard.putNumber("pot 2", switches.getPot2());
		// SmartDashboard.putNumber("pot 3", switches.getPot3());
		// SmartDashboard.putNumber("pot 4", switches.getPot4());
		// SmartDashboard.putNumber("pot side", switches.getPotSide());
//		SmartDashboard.putNumber("left encoder", Robot.drive.motors[2].getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Right encoder", Robot.drive.motors[0].getSelectedSensorPosition(0));
		SmartDashboard.putNumber("match timer per part of match",(int)DriverStation.getInstance().getMatchTime());

		if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kForward)) {
			SmartDashboard.putString("intake", "forw");
		} else if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kReverse)) {
			SmartDashboard.putString("intake", "rev");
		}
		if (arm.getArmHalZeroFront()) {
			arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, 0, 20);
		} else if (arm.getArmHalZeroBack()) {
			arm.bottomMotor.setSelectedSensorPosition(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, 0, 20);
		}
		if (lift.isSecondStageAtBottom()) {
			lidarOffset = lidar.getSample();
		}
		if (lidarCount == 10) {
			lidarValue = lidar.getSample() - lidarOffset;
			lidarCount = 0;
		}
		lidarCount++;
		SmartDashboard.putNumber("adj Lidar", lidarValue);
		SmartDashboard.putBoolean("Can Flip", lift.canFlip());
		SmartDashboard.putNumber("Arm Current", arm.bottomMotor.getOutputCurrent());
		SmartDashboard.putBoolean("ir", arm.getInfraredSensor());
//		if (lift.isCarriageAtTop() && lift.isSecondStageAtTop()) {
//			 lift.mainLiftMotor.setSelectedSensorPosition(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,0, 20);
//		}

		prevStateIntakeToggle = oi.operator.openIntakeToggleBB();
		prevStateIntakeToggle2 = oi.operator.openIntakeToggle();
		prevStateFlipAndIntakeHigh = oi.operator.flipAndIntakeHigh();
		prevStateFlipAndIntakeLow = oi.operator.flipAndIntakeLow();
		prevStateFlipAndStore = oi.operator.flipAndStore();
	}

	// if (DriverStation.getInstance().getMatchTime() <= 30.0 &&
	// DriverStation.getInstance().getMatchTime() >= 29.0) {
	// hasEndgameStarted = true;
	// if (hasEndgameStarted) {
	// dio.setPWMRate(0.03);
	// hasEndgameStarted = false;
	// }
	// }
	// if (intake.isRunning() == true) {
	// if (arm.getInfraredSensor() == true) {
	// dio.setPWMRate(0.04);;
	// } else if (arm.getInfraredSensor() == false) {
	// dio.setPWMRate(0.05);
	// }
	// } else if (intake.isRunning() == false) {
	// if (arm.getInfraredSensor() == true) {
	// dio.setPWMRate(0.06);
	// } else if (arm.getInfraredSensor() == false) {
	// dio.setPWMRate(0.07);
	// }
	// }
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
		// SmartDashboard.putNumber("TalonRaw",
		// drive.motors[0].getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Talon Enc Distance",
		// drive.getTalonDistanceLeft());
//		SmartDashboard.putBoolean("Infrared", arm.getInfraredSensor());
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

	public void setLEDs(int newState) {
//		ledState = newState;
//		DriverStation.reportWarning("led code " + newState, false);
	}
}
