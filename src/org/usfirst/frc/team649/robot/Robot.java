
package org.usfirst.frc.team649.robot;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.autonomous.autoMaster;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	public static ArmSubsystem arm;
	public static LiftSubsystem lift;
	public static autoMaster automaster;
	public Logger logger;
	public FileHandler local;
	public FileHandler usb;
	public File testForPath1;
	public File testForPath2;
	public Timer matchTimer;
	public double teleOpTime = 135;
	public double autoTime = 15;
	public double currentTime;
	public String conversionTime;
	public double modeTime;

	// prev state variables leave at bottom

	// these two are for buttons not the actual
	public static boolean autoShiftButtonPrevState;
	public static boolean VPidButtonPrevState;

	@Override
	public void robotInit() {
		oi = new OI();
		drive = new DrivetrainSubsystem();
		gyro = new GyroSubsystem();
		arm = new ArmSubsystem();
		lift = new LiftSubsystem();
		automaster = new autoMaster();
		isPIDActive = false;
		k_p = drive.getPIDController().getP();
		k_i = drive.getPIDController().getI();
		k_d = drive.getPIDController().getD();
		tuningConstant = 1;
		logger = Logger.getLogger("robotLog");
		matchTimer = new Timer();
		testForPath1 = new File("/media/sdb1/logdatausb.txt");
		testForPath2 = new File("/media/sdb1/logdatausb.txt");
		try {
			local = new FileHandler("/home/lvuser/logdatabase.txt");
			logger.addHandler(local);
			SimpleFormatter localFormatter = new SimpleFormatter();
			local.setFormatter(localFormatter);
		} catch (SecurityException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {

			usb = new FileHandler("/media/sda1/logdatausb.txt");
			logger.addHandler(usb);
			SimpleFormatter localFormatter = new SimpleFormatter();
			usb.setFormatter(localFormatter);
		} catch (SecurityException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

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
		drive.resetEncoders();
		new DrivetrainPIDCommand(30.0).start();
		logger.setUseParentHandlers(false);
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		if (isAutonomous()) {
			logNewEvent(returnDifferenceInMatchTime() + " " + "I'm In Autonomous Mode!");
		}
		if (oi.operator.PIDTunePhase()) {
			SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
		}

		if (isTuningPID) {
			if (oi.operator.getButton2()) {
				isTuningPID = false;
				drive.getPIDController().setPID(k_p, k_i, k_d);
				new DrivetrainPIDCommand(30).start();
			}

			if (oi.operator.getButton6()) {
				if (tuningConstant == 1) {
					k_p += 0.1;
				} else if (tuningConstant == 2) {
					k_i += 0.1;
				} else if (tuningConstant == 3) {
					k_d += 0.1;
				}
			}
			if (oi.operator.getButton4()) {
				if (tuningConstant == 1) {
					k_p -= 0.1;
				} else if (tuningConstant == 2) {
					k_i -= 0.1;
				} else if (tuningConstant == 3) {
					k_d -= 0.1;
				}
			}
			if (oi.operator.getButton5()) {
				if (tuningConstant == 1) {
					k_p += 0.01;
				} else if (tuningConstant == 2) {
					k_i += 0.01;
				} else if (tuningConstant == 3) {
					k_d += 0.01;
				}
			}
			if (oi.operator.getButton7()) {
				if (tuningConstant == 1) {
					k_p -= 0.01;
				} else if (tuningConstant == 2) {
					k_i -= 0.01;
				} else if (tuningConstant == 3) {
					k_d -= 0.01;
				}
			}
			if (oi.operator.getButton3()) {
				if (tuningConstant < 3) {
					tuningConstant += 1;
				} else {
					tuningConstant = 1;
				}
			}
			if (tuningConstant == 1) {
				SmartDashboard.putString("Currently Tuning", "k_p: " + k_p);
			} else if (tuningConstant == 2) {
				SmartDashboard.putString("Currently Tuning", "k_i: " + k_i);
			} else if (tuningConstant == 3) {
				SmartDashboard.putString("Currently Tuning", "k_d: " + k_d);
			}
			SmartDashboard.putNumber("k_p", k_p);
			SmartDashboard.putNumber("k_i", k_i);
			SmartDashboard.putNumber("k_d", k_d);
		}
		SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
		SmartDashboard.putNumber("k_p", k_p);
		SmartDashboard.putNumber("k_i", k_i);
		SmartDashboard.putNumber("k_d", k_d);

	}

	@Override
	public void teleopInit() {
		logger.setUseParentHandlers(false);
		isAutoShift = true;
		isVPid = true;
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
		drive.resetEncoders();
		lift.resetLiftEncoder();
		new Thread(() -> {

			AxisCamera camera1 = CameraServer.getInstance().addAxisCamera("greenAxisCamera", RobotMap.Camera.axisPort);
			camera1.setResolution(RobotMap.Camera.axisResWidth, RobotMap.Camera.axisResWidth);
			camera1.setFPS(RobotMap.Camera.axisFPS);
			AxisCamera camera2 = CameraServer.getInstance().addAxisCamera("blackAxisCamera", RobotMap.Camera.axis2Port);
			camera2.setResolution(RobotMap.Camera.axis2ResWidth, RobotMap.Camera.axis2ResHeight);
			camera2.setFPS(RobotMap.Camera.axis2FPS);
			AxisCamera camera3 = CameraServer.getInstance().addAxisCamera("whiteAxisCamera", RobotMap.Camera.axis3Port);
			camera3.setResolution(RobotMap.Camera.axis3ResWidth, RobotMap.Camera.axis3ResHeight);
			camera3.setFPS(RobotMap.Camera.axis3FPS);

			CvSink cvSink = CameraServer.getInstance().getVideo("greenAxisCamera");
			CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);

			Mat frame = new Mat();

			while (!Thread.interrupted()) {

				if (oi.operator.switchToCamera1()) {
					System.out.println("Switching to camera 1");
					cvSink = CameraServer.getInstance().getVideo("greenAxisCamera");
				} else if (oi.operator.switchToCamera2()) {
					System.out.println("Switching to camera 2");
					cvSink = CameraServer.getInstance().getVideo("blackAxisCamera");
				} else if (oi.operator.switchToCamera3()) {
					System.out.println("Switching to camera 3");
					cvSink = CameraServer.getInstance().getVideo("whiteAxisCamera");
				}

				cvSink.grabFrame(frame);
				outputStream.putFrame(frame);

			}
		}).start();

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		if (oi.driveJoystickHorizontal.getRawButton(1))
		{
			logNewEvent(returnDifferenceInMatchTime() + " " + "Button Test!");
		}
		checkAutoShiftToggle();
		checkVbusToggle();

		// if(!isAutoShift || oi.driver.forceLowGear()){
		// //manual shift
		// }else{
		// //auto shift
		// }
		SmartDashboard.putBoolean("is VPID runnig", isVPid);
		
		lift.getLiftState();
		double liftJoy = oi.operator.getOperatorY();
		if(lift.getLiftState() == LiftSubsystem.LiftStateConstants.LOWEST_STATE){
			if(liftJoy<0){
				liftJoy = 0;
			}
		}else if(lift.getLiftState() == LiftSubsystem.LiftStateConstants.CARRIAGE_HIGH_SECOND_HIGH){
			if(liftJoy>0.15){
				liftJoy=0;
			}
		}
		lift.setLift(liftJoy);
		if(oi.operator.getArmUp()){
			arm.setArm(-oi.operatorJoystick.getRawAxis(2));
		}else if(oi.operator.getArmDown()){
			arm.setArm(oi.operatorJoystick.getRawAxis(2));
		}else{
			arm.setArm(0.0);
		}
		SmartDashboard.putNumber("SLider", -oi.operatorJoystick.getRawAxis(2));
		
		
//		double joyXVal = -Robot.oi.driver.getRotation();
//		double joyYVal = Robot.oi.driver.getForward();
//		if (!isVPid || oi.driver.isVBusOveridePush() || ((Math.abs(joyXVal) < 0.1) && joyYVal == 0)) {
//			if (joyXVal > 0) {
//				joyXVal = Math.pow(joyXVal, 2);
//			} else {
//				joyXVal = -Math.pow(Math.abs(joyXVal), 2);
//			}
//			SmartDashboard.putBoolean("is in Vbus", true);
//			drive.driveFwdRotate(joyYVal, joyXVal, true);
//		} else if (Math.abs(joyXVal) < 0.1 && Math.abs(joyYVal) < 0.15) {
//			drive.driveFwdRotate(joyYVal, joyXVal, true);
//		} else {
//			SmartDashboard.putBoolean("is in Vbus", false);
//			if (joyYVal > 0) {
//				joyYVal = Math.pow(Math.abs(joyYVal), DrivetrainSubsystem.VPIDConstants.Y_COMPONENT_EXP);
//			} else {
//				joyYVal = -Math.pow(Math.abs(joyYVal), DrivetrainSubsystem.VPIDConstants.Y_COMPONENT_EXP);
//			}
//			if (joyXVal > 0) {
//				if (joyXVal > 0.3) {
//					joyXVal = Math.pow(Math.abs(joyXVal), DrivetrainSubsystem.VPIDConstants.X_COMPONENT_HIGH_EXP);
//				} else {
//					joyXVal = Math.pow(Math.abs(joyXVal), DrivetrainSubsystem.VPIDConstants.X_COMPONENT_LOW_EXP);
//				}
//
//			} else {
//				if (joyXVal < -0.1) {
//					joyXVal = -Math.pow(Math.abs(joyXVal), DrivetrainSubsystem.VPIDConstants.X_COMPONENT_HIGH_EXP);
//				} else {
//					joyXVal = -Math.pow(Math.abs(joyXVal), DrivetrainSubsystem.VPIDConstants.X_COMPONENT_LOW_EXP);
//				}
//			}
//			drive.driveFwdRotate(joyYVal, joyXVal, false);
//		}
//		// drive.rawDrive(oi.driveJoystickVertical.getY(),
//		// oi.driveJoystickVertical.getY());
//		// drive.rawDrive(0.3, 0.3);
//		// drive.driveFwdRotate(oi.driver.getForward(), oi.driver.getRotation(), true);
//		// these are checking the previous state of a variable make sure this is at the
//		// bottom
//		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
//		VPidButtonPrevState = oi.driver.switchToVbus();

//		if (oi.operator.PIDTunePhase()) {
//			SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
//			isTuningPID = true;
//		}
//
//		if (isTuningPID) {
//			if (oi.operator.getButton2()) {
//				isTuningPID = false;
//				drive.getPIDController().setPID(k_p, k_i, k_d);
//				new DrivetrainPIDCommand(30).start();
//			}
//
//			if (oi.operator.getButton6()) {
//				if (tuningConstant == 1) {
//					k_p += 0.1;
//				} else if (tuningConstant == 2) {
//					k_i += 0.1;
//				} else if (tuningConstant == 3) {
//					k_d += 0.1;
//				}
//			}
//			if (oi.operator.getButton4()) {
//				if (tuningConstant == 1) {
//					k_p -= 0.1;
//				} else if (tuningConstant == 2) {
//					k_i -= 0.1;
//				} else if (tuningConstant == 3) {
//					k_d -= 0.1;
//				}
//			}
//			if (oi.operator.getButton5()) {
//				if (tuningConstant == 1) {
//					k_p += 0.01;
//				} else if (tuningConstant == 2) {
//					k_i += 0.01;
//				} else if (tuningConstant == 3) {
//					k_d += 0.01;
//				}
//			}
//			if (oi.operator.getButton7()) {
//				if (tuningConstant == 1) {
//					k_p -= 0.01;
//				} else if (tuningConstant == 2) {
//					k_i -= 0.01;
//				} else if (tuningConstant == 3) {
//					k_d -= 0.01;
//				}
//			}
//			if (oi.operator.getButton3()) {
//				if (tuningConstant < 3) {
//					tuningConstant += 1;
//				} else {
//					tuningConstant = 1;
//				}
//			}
//			if (tuningConstant == 1) {
//				SmartDashboard.putString("Currently Tuning", "k_p: " + k_p);
//			} else if (tuningConstant == 2) {
//				SmartDashboard.putString("Currently Tuning", "k_i: " + k_i);
//			} else if (tuningConstant == 3) {
//				SmartDashboard.putString("Currently Tuning", "k_d: " + k_d);
//			}
//		}
//		// drive.driveFwdRotate(oi.driver.getForward(), -oi.driver.getRotation(), true);
		updateSmartDashboardTesting();
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
//		SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
//		SmartDashboard.putNumber("k_p", k_p);
//		SmartDashboard.putNumber("k_i", k_i);
//		SmartDashboard.putNumber("k_d", k_d);
//		SmartDashboard.putNumber("TalonRaw", drive.motors[0].getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Talon Enc Distance", drive.getTalonDistanceLeft());
//		SmartDashboard.putBoolean("Infrared", arm.getInfraredSensor());
//		SmartDashboard.putNumber("Talon Enc Distance Left", drive.getTalonDistanceLeft());
//		SmartDashboard.putNumber("Talon Enc Distance Right", drive.getTalonDistanceRight());
//
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
		SmartDashboard.putBoolean("is Second Stage at Bottom", lift.isSecondStageAtBottom());
		SmartDashboard.putBoolean("is Second Stage at Top", lift.isSecondStageAtTop());
		SmartDashboard.putBoolean("is Carriage at Bottom", lift.isCarriageAtBottom());
		SmartDashboard.putBoolean("is Carriage at Top", lift.isCarriageAtTop());
		SmartDashboard.putNumber("Lift Raw", lift.getRawLift());
		SmartDashboard.putNumber("Lift Scaled Distance", lift.getLiftDistance());
		SmartDashboard.putNumber("Lidar", lift.getLidarValue());
		int state = lift.getLiftState();
		if(state == 1){
			SmartDashboard.putString("Lift State (carriage/second)", "Low Low");
		}else if(state == 2){
			SmartDashboard.putString("Lift State (carriage/second)", "Low Mid");
		}else if(state == 3){
			SmartDashboard.putString("Lift State (carriage/second)", "Low High");
		}else if(state == 4){
			SmartDashboard.putString("Lift State (carriage/second)", "Mid High");
		}else if(state == 5){
			SmartDashboard.putString("Lift State (carriage/second)", "High High");
		}else if(state == 6){
			SmartDashboard.putString("Lift State (carriage/second)", "High Mid");
		}else if(state == 7){
			SmartDashboard.putString("Lift State (carriage/second)", "High Low");
		}else if(state == 8){
			SmartDashboard.putString("Lift State (carriage/second)", "Mid Low");
		}
	}

	private void updateSmartDashboardComp() {

	}

	@Override
	public void testPeriodic() {

	}
	public void logNewEvent(String eventToLog) {
		logger.info(eventToLog);
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
