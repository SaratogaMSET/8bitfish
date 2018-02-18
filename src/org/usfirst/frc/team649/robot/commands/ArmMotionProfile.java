package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMotionProfile extends Command {
	int value;
    public ArmMotionProfile(int encoderValue) {
    	value = encoderValue;
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.isArmPidRunning = true;
    	Robot.arm.setArmBrake(false);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.arm.bottomMotor.set(ControlMode.MotionMagic, value);
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.arm.bottomMotor.getSelectedSensorPosition(0)-value)<ArmSubsystem.ArmConstants.RAW_ABS_TOL || (Robot.arm.getVel() < 15 &&Math.abs(Robot.arm.bottomMotor.getSelectedSensorPosition(0)-value)<50);
    }

    // Called once after isFinished returns true
    protected void end() {
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
