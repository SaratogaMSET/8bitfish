
package org.usfirst.frc.team649.robot;

import org.usfirst.frc.team649.autonomous.autoMaster;
import org.usfirst.frc.team649.robot.commands.DistanceTalonPID;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
	public static autoMaster automaster;
	
	
	//prev state variables leave at bottom
	
	//these two are for buttons not the actual 
	public static boolean autoShiftButtonPrevState;
	public static boolean VPidButtonPrevState;
	
	
	@Override
	public void robotInit() {
		oi = new OI();
		drive = new DrivetrainSubsystem();
		gyro = new GyroSubsystem();
		automaster = new autoMaster();
		isPIDActive = false;
		k_p = drive.getPIDController().getP();
		k_i = drive.getPIDController().getI();
		k_d = drive.getPIDController().getD();
		tuningConstant = 1;
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
		automaster.autoDecider();
		drive.resetEncoders();
		new DrivetrainPIDCommand(30.0).start();
	}
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		
		if (oi.operator.PIDTunePhase()) {
				SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
		}
		
		if (isTuningPID) {
			if (oi.operator.getButton2()) {
				isTuningPID = false;
				drive.getPIDController().setPID(k_p, k_i, k_d);
				new DrivetrainPIDCommand(30).start()
				;
			}
			
			if(oi.operator.getButton6()) {
				if (tuningConstant == 1) {
					k_p += 0.1;
				} else if (tuningConstant == 2) {
					k_i += 0.1;
				} else if (tuningConstant == 3) {
					k_d += 0.1;
				}
			}
			if(oi.operator.getButton4()) {
				if (tuningConstant == 1) {
					k_p -= 0.1;
				} else if (tuningConstant == 2) {
					k_i -= 0.1;
				} else if (tuningConstant == 3) {
					k_d -= 0.1;
				}
			}
			if(oi.operator.getButton5()) {
				if (tuningConstant == 1) {
					k_p += 0.01;
				} else if (tuningConstant == 2) {
					k_i += 0.01;
				} else if (tuningConstant == 3) {
					k_d += 0.01;
				}
			}
			if(oi.operator.getButton7()) {
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
		isAutoShift = true;
		isVPid = true;
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
		drive.resetEncoders();
		
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		checkAutoShiftToggle();
		checkVbusToggle();
		
//		if(!isAutoShift || oi.driver.forceLowGear()){
//			//manual shift
//		}else{
//			//auto shift
//		}
//		double joyXVal = Robot.oi.driver.getRotation();
//		double joyYVal = Robot.oi.driver.getForward();
//		if(!isVPid || oi.driver.isVBusOveridePush()||(Math.abs(joyXVal)<0.2 && joyYVal == 0)){
//			drive.driveFwdRotate(joyYVal, joyXVal, true);
//		}else{
//			
//		}
//		drive.rawDrive(oi.driveJoystickVertical.getY(), oi.driveJoystickVertical.getY());
//		drive.rawDrive(0.3, 0.3);
//		drive.driveFwdRotate(oi.driver.getForward(), oi.driver.getRotation(), true);
//		SmartDashboard.putNumber("TalonRaw", drive.motors[0].getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Talon Enc Distance", drive.getTalonDistanceLeft());
//		//these are checking the previous state of a variable make sure this is at the bottom			
//		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
//		VPidButtonPrevState = oi.driver.switchToVbus();
		if (oi.operator.PIDTunePhase()) {
			SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
			isTuningPID = true;
		}
	
	if (isTuningPID) {
		if (oi.operator.getButton2()) {
			isTuningPID = false;
			drive.getPIDController().setPID(k_p, k_i, k_d);
			new DrivetrainPIDCommand(30).start();
		}
		
		if(oi.operator.getButton6()) {
			if (tuningConstant == 1) {
				k_p += 0.1;
			} else if (tuningConstant == 2) {
				k_i += 0.1;
			} else if (tuningConstant == 3) {
				k_d += 0.1;
			}
		}
		if(oi.operator.getButton4()) {
			if (tuningConstant == 1) {
				k_p -= 0.1;
			} else if (tuningConstant == 2) {
				k_i -= 0.1;
			} else if (tuningConstant == 3) {
				k_d -= 0.1;
			}
		}
		if(oi.operator.getButton5()) {
			if (tuningConstant == 1) {
				k_p += 0.01;
			} else if (tuningConstant == 2) {
				k_i += 0.01;
			} else if (tuningConstant == 3) {
				k_d += 0.01;
			}
		}
		if(oi.operator.getButton7()) {
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
	private void checkAutoShiftToggle(){
		//on release 
		if(!oi.driver.switchToNormalShift() && autoShiftButtonPrevState){
			isAutoShift = !isAutoShift;
		}
	}
	private void checkVbusToggle(){
		if(!oi.driver.switchToVbus() && VPidButtonPrevState){
			isVPid = !isVPid;
		}
	}
	private void updateSmartDashboardTesting(){
		SmartDashboard.putNumber("TalonRaw", drive.motors[0].getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Talon Enc Distance", drive.getTalonDistanceLeft());
	}
	private void updateSmartDashboardComp(){
		
	}

	@Override
	public void testPeriodic() {
		
	}
}
