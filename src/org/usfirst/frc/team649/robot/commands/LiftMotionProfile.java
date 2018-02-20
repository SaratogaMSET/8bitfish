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
public class LiftMotionProfile extends Command {
	int value;
	Timer doneTime;
	Timer timeout;
	int donePos;
    public LiftMotionProfile(int encoderValue) {
    	value = encoderValue;
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("ran is fin", false);
//		Robot.isArmPidRunning = true;
//    	Robot.arm.setArmBrake(false);
    	doneTime = new Timer();
    	donePos = 0;
    	timeout.start();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.lift.mainLiftMotor.set(ControlMode.MotionMagic, value);
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(doneTime.get() > 0.15){
    		return true;
    		
    	}else if(!(doneTime.get() == 0) && Math.abs(donePos-Robot.lift.mainLiftMotor.getSelectedSensorPosition(0)) > 100){
    		doneTime.stop();
    		doneTime.reset();
    	}else if(Math.abs(Robot.lift.mainLiftMotor.getSelectedSensorPosition(0) - value)< 100 && doneTime.get() == 0){
    		doneTime.start();
    	} else if (timeout.get() > 5) {
    		return true;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putBoolean("ran is fin", true);
    	Robot.lift.mainLiftMotor.set(ControlMode.PercentOutput, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
