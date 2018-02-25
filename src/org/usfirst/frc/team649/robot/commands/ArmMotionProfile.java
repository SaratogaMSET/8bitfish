package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmMotionProfile extends Command {
	int value;
	Timer doneTime;
	int donePos;
	int state;
    public ArmMotionProfile(int encoderValue,int state) {
    	value = encoderValue;
    	
        this.state = state;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("ran is fin", false);
		Robot.isArmPidRunning = true;
    	Robot.arm.setArmBrake(false);
    	doneTime = new Timer();
    	donePos = 0;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.arm.bottomMotor.set(ControlMode.MotionMagic, value);
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//    	if(doneTime.get() > 0.15){
//    		return true;
//    		
//    	}else if(!(doneTime.get() == 0) && Math.abs(donePos-Robot.arm.getArmRaw()) > 20){
//    		doneTime.stop();
//    		doneTime.reset();
//    	}else if(Math.abs(Robot.arm.getArmRaw() - value)< ArmSubsystem.ArmConstants.RAW_ABS_TOL && doneTime.get() == 0){
//    		donePos = Robot.arm.getArmRaw();
//    		doneTime.start();
//    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	if(Robot.armState > 0){
    		if(Robot.armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP){
    			Robot.armState--;
    		}else{
    			Robot.armState++;
    		}
    	}else{
    			Robot.armState--;
    	}
    	SmartDashboard.putBoolean("ran is fin", true);
		Robot.isArmPidRunning = false;
    	Robot.arm.setArmBrake(true);
    	Robot.arm.bottomMotor.set(ControlMode.PercentOutput, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
