
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
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.CommandGroups.IntakeWithWheelsAndClose;
import org.usfirst.frc.team649.robot.commands.AngleTalonPID;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.Diagnostic;
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
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.robot.util.Lidar;
import org.usfirst.frc.team649.test.AutoTestCommand;
import org.usfirst.frc.team649.test.Square;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

public class Robot extends TimedRobot {

	public static OI oi;
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
	public static boolean drivePIDRunning;
	public Logger logger;
	public FileHandler local;
	public FileHandler usb;
	public File testForPath1;
	public File testForPath2;
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

	// prev state variables leave at bottom

	// these two are for buttons not the actual
	public static boolean autoShiftButtonPrevState;
	public static boolean VPidButtonPrevState;
	
	public static boolean liftManualPrevState;
	public static boolean armManualPrevState;
	public static boolean prevStateFlipArm;

	@Override
	public void robotInit() {
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
//		automaster = new autoMaster();
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
		time = new Timer();
		timeAccel = new Timer();
		driveAccel = 0;
		driveVel = 0;
		prevDriveVel = 0;
		liftState = 2;
		armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
		rightDTMaxVel = 0;
		leftDTMaxVel = 0;
		timesCalled = 0;
//		logger = Logger.getLogger("robotLog");
//		matchTimer = new Timer();
//		testForPath1 = new File("/media/sdb1/logdatausb.txt");
//		testForPath2 = new File("/media/sdb1/logdatausb.txt");
//		try {
//			local = new FileHandler("/home/lvuser/logdatabase.txt");
//			logger.addHandler(local);
//			SimpleFormatter localFormatter = new SimpleFormatter();
//			local.setFormatter(localFormatter);
//		} catch (SecurityException e) {
//			e.printStackTrace();
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
//		try {
//
//			usb = new FileHandler("/media/sda1/logdatausb.txt");
//			logger.addHandler(usb);
//			SimpleFormatter localFormatter = new SimpleFormatter();
//			usb.setFormatter(localFormatter);
//		} catch (SecurityException e) {
//			e.printStackTrace();
//		} catch (IOException e) {
//			e.printStackTrace();
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
		arm.setArmBrake(true);
		drive.resetEncoders();
//		new DrivetrainMotionProfileIn(50.0).start();
		liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
		drive.shift(true);
		for(int i = 0; i < 4; i++) {
			drive.motors[i].setNeutralMode(NeutralMode.Brake);
			drive.motors[i].configMotionAcceleration(9000, timeoutMs);
			drive.motors[i].configMotionCruiseVelocity(18000, timeoutMs);
			drive.motors[i].config_kP(0, 1.5, Robot.timeoutMs);
			drive.motors[i].config_kI(0, 0, Robot.timeoutMs);
			drive.motors[i].config_kD(0, 0.9, Robot.timeoutMs);
		}
//		drive.changeBrakeCoast(true);
//		if(program == 1) {
//			new CenterSwitchRight().start();
//		} else if (program == 2) {
//			new CenterSwitchLeft().start();
//		}
		new RightFarScale().start();
//		new LeftScale().start();
//		new RightSwitch().start();
//		new DrivetrainMotionProfileIn(230).start();
//		new GyroStraightPID(200).start();
//		new GyroPID(90).start();
//		int liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
//		new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE,liftState).start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		if (isAutonomous()) {
			//logNewEvent(returnDifferenceInMatchTime() + " " + "I'm In Autonomous Mode!");
		}
//		updateSmartDashboardTesting();
		SmartDashboard.putNumber("arm", arm.getArmRaw());
		updateSmartDashboardTesting();


		

	}

	@Override
	public void teleopInit() {
		armVelMax = 0;
		intakeTimer.start();
//		gyro.resetGyro();
//		logger.setUseParentHandlers(false);
//		drive.changeBrakeCoast(false);
		isAutoShift = true;
		maxAccelDrive = 0;
		isVPid = true;
		isArmPidRunning = false;
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
//		arm.resetEncoder();
		drive.resetEncoders();
		lift.resetLiftEncoder();
		accelTimer.start();
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
		isLiftStall  = false;	
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
//		new SetCompressorCommand(true).start();
		arm.bottomMotor.setSelectedSensorPosition(0, 0, 20);
		drive.changeBrakeCoast(true);
		for(int i = 0; i < 4; i++) {
			drive.motors[i].setNeutralMode(NeutralMode.Brake);
		}
		drive.shift(true);
		 new AutoTestCommand().start();
//		 drive.shift(true);
			for(int i = 0; i < 4; i++) {
				drive.motors[i].setNeutralMode(NeutralMode.Brake);
				drive.motors[i].configMotionAcceleration(9000, timeoutMs);
				drive.motors[i].configMotionCruiseVelocity(18000, timeoutMs);
				drive.motors[i].config_kP(0, 1.5, Robot.timeoutMs);
				drive.motors[i].config_kI(0, 0, Robot.timeoutMs);
				drive.motors[i].config_kD(0, 0.9, Robot.timeoutMs);
			}

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

//		if (oi.driveJoystickHorizontal.getRawButton(1))
//		{
//			logNewEvent(returnDifferenceInMatchTime() + " " + "Button Test!");
//		}
		checkAutoShiftToggle();
		checkVbusToggle();
		updateSmartDashboardTesting();
//		teleopRun();
//		
////		drive.rawDrive(0.5, 0.5);
//		updateSmartDashboardTesting();
//		
//		drive.rawDrive(0.5, 0.5);
//		teleopRun();
//		if(lidarCount == 12){
//			SmartDashboard.putNumber("Lidar", lidar.getSample());
//			lidarCount = 0;
//		}
//		lidarCount ++;
	}
	public void teleopRun(){
		if(oi.driver.shiftUp()){
			drive.shift(true);
		}else{
			drive.shift(false);
		}
		double rot = -oi.driver.getRotation();
		if(rot > 0){
			rot = Math.pow(rot, 2);
		}else{
			rot = -Math.pow(Math.abs(rot), 2);
		}
		drive.driveFwdRotate(oi.driver.getForward(), rot, true);
		

//		if(oi.operator.getIntakeState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE && liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,liftState).start();
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR && armState != ArmSubsystem.ArmStateConstants.INTAKE_FRONT && armState != ArmSubsystem.ArmStateConstants.INTAKE_REAR){
//				if(armIsFront){
//					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT,armState).start();
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR,armState).start();
//
//				}
//			}
//		}else if(oi.operator.getStoreState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE && liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,liftState).start();
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR && armState != ArmSubsystem.ArmStateConstants.STORE_FRONT && armState != ArmSubsystem.ArmStateConstants.STORE_REAR){
//				if(armIsFront){
//					timesCalled++;
//					SmartDashboard.putNumber("times Call", timesCalled);
//					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT,armState).start();
//
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR,armState).start();
//
//				}
//			}
//			
//		}else if(oi.operator.getSwitchState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_SWITCH_STATE && liftState != LiftSubsystem.LiftStateConstants.SWITCH_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_SWITCH_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.SWITCH_STATE,liftState);
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR && armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR){
//				if(armIsFront){
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT,armState).start();
//
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR,armState).start();
//
//				}
//			}
//		}else if(oi.operator.getScaleLowState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE && liftState != LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE,liftState).start();
//				SmartDashboard.putBoolean("got in", true);
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR && armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR){
//				if(armIsFront){
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT,armState).start();
//
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR,armState).start();
//
//				}
//			}
//		}else if(oi.operator.getScaleMidState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE && liftState != LiftSubsystem.LiftStateConstants.MID_SCALE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE,liftState).start();
//
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR && armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR){
//				if(armIsFront){
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT,armState).start();
//
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR,armState).start();
//
//				}
//			}
//		}else if(oi.operator.getScaleHighState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE && liftState != LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,liftState).start();
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR && armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT && armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR){
//				if(armIsFront){
//					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,armState).start();
//
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR,armState).start();
//
//				}
//			}
//			
//		}else if(oi.operator.getArmUpSmall()){
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP && arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ < 0){
//				armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
//				if(arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID){
//					armIsFront = true;
//				
//				}else { 
//					armIsFront = false;
//				}
//				customArmPos = arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ;
//				new ArmMotionProfile(customArmPos,armState).start();
//
//			}
//		}else if(oi.operator.getArmDownSmall()){
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN && arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.INTAKE_REAR){
//				armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
//				if(arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID){
//					armIsFront = true;
//				}else { 
//					armIsFront = false;
//				}
//				customArmPos = arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ;
//				new ArmMotionProfile(customArmPos,armState).start();
//			}
//		}else if(oi.operator.getLiftUpSmall()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP && lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST < LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP;
//				customLiftPos = (int)lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
//				new LiftMotionProfile(customLiftPos,liftState).start();
//			}
//			
//		}else if(oi.operator.getLiftDownSmall()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN && lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST > 0){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN;
//				customLiftPos = (int)lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
//				new LiftMotionProfile(customLiftPos ,liftState).start();
//			}
//		}else if(oi.operator.getExchangeState()){
//			if(liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE && liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE){
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,liftState).start();
//			}
//			if(armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT && armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR && armState != ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT && armState != ArmSubsystem.ArmStateConstants.EXCHANGE_REAR){
//				if(armIsFront){
//					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT,armState).start();
//				}else{
//					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR,armState).start();
//
//				}
//			}
//		}else if(oi.operator.flipArm()){
////			if(lift.isCarriageAtBottom()){ //temp
//				if(!prevStateFlipArm && lift.getRawLift() < LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE){
//					if(armIsFront){
//						if(armState == ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT ||armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR,armState).start();
//						}else if(armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP || armState == ArmSubsystem.ArmStateConstants.CUSTOM){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos,armState).start();
//						}else if(armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT||armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR,armState).start();
//						}else if(armState == ArmSubsystem.ArmStateConstants.MID_DROP_FRONT||armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR,armState).start();
//						}else if(armState == ArmSubsystem.ArmStateConstants.STORE_FRONT || armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR,armState).start();
//						}else if(armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT || armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR,armState).start();
//						}
//					}else{
//						if(armState == ArmSubsystem.ArmStateConstants.EXCHANGE_REAR ||armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT,armState).start();
//						}else if(armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN || armState == ArmSubsystem.ArmStateConstants.CUSTOM){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos,armState).start();
//
//						}else if(armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR||armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,armState).start();
//
//						}else if(armState == ArmSubsystem.ArmStateConstants.MID_DROP_REAR||armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT,armState).start();
//
//						}else if(armState == ArmSubsystem.ArmStateConstants.STORE_REAR || armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT,armState).start();
//
//						}else if(armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR || armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR){
//							armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
//							new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT,armState).start();
//
//						}
//					}
//				}
////			}
//		}else if(oi.operator.isManual()){
//			liftState = LiftSubsystem.LiftStateConstants.CUSTOM_STATE;
//			armState = ArmSubsystem.ArmStateConstants.CUSTOM;
//			customLiftPos = (int) lift.getRawLift();
//			customArmPos = (int) arm.getArmRaw();
//			double liftJoy = oi.operator.getManualLift();
//			double newLift = liftJoy;
//			if(lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE){
//				if(liftJoy<0){
//					newLift = 0;
//				}
//				liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
//
//			}else if(lift.getLiftState() == LiftSubsystem.LiftHalConstants.CARRIAGE_HIGH_SECOND_HIGH){
//				if(liftJoy>0.185){
//					newLift=0.185;
//				}
//			}else if(liftJoy == 0){
//				newLift = 0.185;
//			}else {
//				newLift = liftJoy;
//			}
//			lift.setLift(newLift);
//			double armJoy = oi.operator.getManualArm();
//			if(armJoy == 0) {
//				if (time.get() > 0.3) {
//					arm.setArmBrake(true);
//				}
//			} else {
//				arm.setArm(armJoy/1.5);
//				arm.setArmBrake(false);
//				time.reset();
//			}				
//		}else{
//			if(liftState == LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE){
//				lift.setLift(0);
//			}else if(liftState == LiftSubsystem.LiftStateConstants.SWITCH_STATE){
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.SWITCH_STATE);
//			}else if(liftState == LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE){
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE);
//			}else if(liftState == LiftSubsystem.LiftStateConstants.MID_SCALE_STATE){
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE);
//			}else if(liftState == LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE){
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE);
//			}else if(liftState == LiftSubsystem.LiftStateConstants.CUSTOM_STATE){
//				lift.setLiftMotion(customLiftPos);
//			}
//			if(!isArmPidRunning){
//				arm.setArmBrake(true);
//			}
//		}
		if(oi.operator.deployOnlyWheels()){
			new RunIntakeWheels(-1).start();
		}else if(oi.operator.deployWithWheelsAndOpen()){
			new DeployWithWheelsAndIntake().start();;
		}else if(oi.operator.openIntake()){
			new SetIntakePistons(true).start();
		}else if(oi.operator.runIntakeWithWheelsClosed()){
			new IntakeWithWheelsAndClose().start();
		}else if(oi.operator.closeIntake()){
			new SetIntakePistons(false).start();
		}else{
			intake.setIntakeMotors(0, 0);
		}
		prevStateFlipArm = oi.operator.flipArm();
		if(arm.getArmRaw() > (ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT +ArmSubsystem.ArmEncoderConstants.INTAKE_REAR)/2){
			armIsFront = true;
		}else{
			armIsFront = false;
		}
		if(lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE){
			lift.resetLiftEncoder();
		}
//		SmartDashboard.putNumber("State", liftState);
//		SmartDashboard.putNumber("arm state", armState);
//		SmartDashboard.putBoolean("isarmfront", armIsFront);
//		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
//		SmartDashboard.putNumber("arm raw", arm.getArmRaw());
//		SmartDashboard.putBoolean("isCarriageAtBot", lift.isCarriageAtBottom());
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
	private void updateSmartDashboardTesting(){
		SmartDashboard.putNumber("Gyro Val", gyro.getGyroAngle());
//		SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
//		SmartDashboard.putNumber("k_p", k_p);
//		SmartDashboard.putNumber("k_i", k_i);
//		SmartDashboard.putNumber("k_d", k_d);
//		SmartDashboard.putNumber("ARM ENCODER", arm.bottomMotor.getSelectedSensorPosition(0));
////		SmartDashboard.putNumber("TalonRaw", drive.motors[0].getSensorCollection().getQuadraturePosition());
////		SmartDashboard.putNumber("Talon Enc Distance", drive.getTalonDistanceLeft());
////		SmartDashboard.putBoolean("Infrared", arm.getInfraredSensor());
//		SmartDashboard.putNumber("Talon Enc Distance Left", drive.motors[0].getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Talon Enc Distance Right", drive.motors[2].getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("LiftJoy", oi.operator.getOperatorY());
//		SmartDashboard.putNumber("WINCH RPM", lift.mainLiftMotor.getSelectedSensorVelocity(0));
//		
//		if(accelTimer.get() > 0.1){
//			SmartDashboard.putNumber("drive accel", (drive.motors[0].getSelectedSensorVelocity(0)-prevDriveVel)/accelTimer.get());
//			SmartDashboard.putNumber("winch accel",(lift.mainLiftMotor.getSelectedSensorVelocity(0)-prevWinchVel)/accelTimer.get());
//			accelTimer.reset();
//			prevWinchVel = lift.mainLiftMotor.getSelectedSensorVelocity(0);
//			prevDriveVel = drive.motors[0].getSelectedSensorVelocity(0);
//		}
//		SmartDashboard.putNumber("PID error", drive.motors[2].getClosedLoopError(0));
//		SmartDashboard.putNumber("Target Current output", drive.motors[2].getMotorOutputPercent());
//
//		SmartDashboard.putNumber("Left Velocity", drive.motors[0].getSelectedSensorVelocity(0));
//		SmartDashboard.putNumber("Left Dist", drive.motors[0].getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Right Velocity", drive.motors[2].getSelectedSensorVelocity(0));
//		SmartDashboard.putNumber("Right Dist", drive.motors[2].getSelectedSensorPosition(0));
//		SmartDashboard.putBoolean("Bottom Second Stage Hal", !lift.botSecondStageHal.get());
//		SmartDashboard.putBoolean("Bottom Carriage Stage Hal", !lift.botCarriageHal.get());
//		SmartDashboard.putBoolean("Top Second Stage Hal", !lift.topSecondStageHal.get());
//		SmartDashboard.putBoolean("Top Carriage Hal", !lift.topCarriageHal.get());
//		SmartDashboard.putNumber("Carriage Winch Raw", lift.getRawLift());
//		SmartDashboard.putBoolean("is Second Stage at Bottom", lift.isSecondStageAtBottom());
//		SmartDashboard.putBoolean("is Second Stage at Top", lift.isSecondStageAtTop());
//		SmartDashboard.putBoolean("is Carriage at Bottom", lift.isCarriageAtBottom());
//		SmartDashboard.putBoolean("is Carriage at Top", lift.isCarriageAtTop());
//		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
//		SmartDashboard.putNumber("Lift Scaled Distance", lift.getLiftDistance());
////		SmartDashboard.putNumber("Arm Raw", arm.getArmRaw());
//		SmartDashboard.putNumber("Vel arm", arm.bottomMotor.getSelectedSensorVelocity(0));
////		SmartDashboard.putNumber("Arm scaled", arm.getArmPosition());
//		SmartDashboard.putBoolean("is Drive PID Running", isDrivePIDRunning);
//		if(lidarCount == 12){
//			SmartDashboard.putNumber("Lidar", lidar.getSample());
//			lidarCount = 0;
//		}
//		lidarCount ++;
////		SmartDashboard.putNumber("curr", drive.motors[0].getOutputCurrent());
////		SmartDashboard.putNumber("pDP", p.getTotalCurrent());
//		int state = lift.getLiftState();
//		if(state == 1){
//			SmartDashboard.putString("Lift State (carriage/second)", "Low Low");
//		}else if(state == 2){
//			SmartDashboard.putString("Lift State (carriage/second)", "Low Mid");
//		}else if(state == 3){
//			SmartDashboard.putString("Lift State (carriage/second)", "Low High");
//		}else if(state == 4){
//			SmartDashboard.putString("Lift State (carriage/second)", "Mid High");
//		}else if(state == 5){
//			SmartDashboard.putString("Lift State (carriage/second)", "High High");
//		}else if(state == 6){
//			SmartDashboard.putString("Lift State (carriage/second)", "High Mid");
//		}else if(state == 7){
//			SmartDashboard.putString("Lift State (carriage/second)", "High Low");
//		}else if(state == 8){
//			SmartDashboard.putString("Lift State (carriage/second)", "Mid Low");
//		}
//		
//		SmartDashboard.putNumber("Scaling Factor", drive.scalingFactor);
//		SmartDashboard.putNumber("Wheel Size", drive.wheelSize);
//		SmartDashboard.putNumber("P", gyro.getP());
//		SmartDashboard.putNumber("I", gyro.getI());
//		SmartDashboard.putNumber("D", gyro.getD());
//		SmartDashboard.putBoolean("is Second Stage at Bottom", lift.isSecondStageAtBottom());
//		SmartDashboard.putBoolean("is Second Stage at Top", lift.isSecondStageAtTop());
//		SmartDashboard.putBoolean("is Carriage at Bottom", lift.isCarriageAtBottom());
//		SmartDashboard.putBoolean("is Carriage at Top", lift.isCarriageAtTop());
//		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
//		SmartDashboard.putNumber("Lift Scaled Distance", lift.getLiftDistance());
//		SmartDashboard.putNumber("Arm Raw", arm.getArmRaw());
//		SmartDashboard.putNumber("Vel arm", arm.bottomMotor.getSelectedSensorVelocity(0));
////		SmartDashboard.putNumber("Arm scaled", arm.getArmPosition());
//		if(lidarCount == 12){
//			SmartDashboard.putNumber("Lidar", lidar.getSample());
//			lidarCount = 0;
//		}
////		SmartDashboard.putNumber("Current 11", PDPJNI.getPDPChannelCurrent(11, 0));
//		lidarCount ++;
////		SmartDashboard.putNumber("curr", drive.motors[0].getOutputCurrent());
////		SmartDashboard.putNumber("pDP", p.getTotalCurrent());
//		int state = lift.getLiftState();
//		if(state == 1){
//			SmartDashboard.putString("Lift State (carriage/second)", "Low Low");
//		}else if(state == 2){
//			SmartDashboard.putString("Lift State (carriage/second)", "Low Mid");
//		}else if(state == 3){
//			SmartDashboard.putString("Lift State (carriage/second)", "Low High");
//		}else if(state == 4){
//			SmartDashboard.putString("Lift State (carriage/second)", "Mid High");
//		}else if(state == 5){
//			SmartDashboard.putString("Lift State (carriage/second)", "High High");
//		}else if(state == 6){
//			SmartDashboard.putString("Lift State (carriage/second)", "High Mid");
//		}else if(state == 7){
//			SmartDashboard.putString("Lift State (carriage/second)", "High Low");
//		}else if(state == 8){
//			SmartDashboard.putString("Lift State (carriage/second)", "Mid Low");
//		}
	}

	private void updateSmartDashboardComp() {

	}

	@Override
	public void testPeriodic() {

	}
//	public void logNewEvent(String eventToLog) {
//		logger.info(eventToLog);
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

}
