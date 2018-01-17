
package org.usfirst.frc.team649.robot;

import org.usfirst.frc.team649.autonomous.autoMaster;
import org.usfirst.frc.team649.robot.commands.DistanceTalonPID;
import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


public class Robot extends TimedRobot {

	public static OI oi;
	public static boolean isPIDActive;
	public static boolean isAutoShift;
	public static boolean isVPid;
	public static boolean isHigh;
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
		new DistanceTalonPID(30).start();
	}
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		isAutoShift = true;
		isVPid = true;
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
		
		
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		checkAutoShiftToggle();
		checkVbusToggle();
		
		if(!isAutoShift || oi.driver.forceLowGear()){
			//manual shift
		}else{
			//auto shift
		}
		double joyXVal = Robot.oi.driver.getRotation();
		double joyYVal = Robot.oi.driver.getForward();
		if(!isVPid || oi.driver.isVBusOveridePush()||(Math.abs(joyXVal)<0.2 && joyYVal == 0)){
			drive.driveFwdRotate(joyYVal, joyXVal, true);
		}else{
			
		}
		
		
		
		//these are checking the previous state of a variable make sure this is at the bottom			
		autoShiftButtonPrevState = oi.driver.switchToNormalShift();
		VPidButtonPrevState = oi.driver.switchToVbus();
		
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
	private void updateSmartDashboard(){
		
	}

	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
