package org.usfirst.frc.team649.robot.commands.arm;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmMotionProfileDelayed extends Command {

	int value;
	Timer doneTime;
	int donePos;
	int state;
	boolean firstTime;
	
    public ArmMotionProfileDelayed(int encoderValue,int state) {
    	value = encoderValue;
    	
        this.state = state;
        firstTime = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {

		SmartDashboard.putBoolean("did get here", false);
		
//    	requires(Robot.arm);
//    	if(front){
//    		Robot.armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
//    	}else{
//    		Robot.armState = ArmSubsystem.ArmStateConstants.INTAKE_REAR;
//    	}
    	SmartDashboard.putBoolean("ran is fin", false);
    	state = Robot.armState;
		Robot.isArmPidRunning = true;
    	doneTime = new Timer();
    	donePos = 0;
    	Robot.arm.setArmBrake(false);
    	Robot.intake.setIntakeMotors(-0.12, -0.12);

//    	doneTime.start();
//    	if(state == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT|| state == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR){
//    		Robot.intake.setIntakeMotors(-0.7, -0.7);
//    	}
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.isZero && Robot.lift.isCarriageAtBottom()){
    		if(!firstTime){
    	    	Robot.arm.setArmBrake(false);
    	    	Robot.armState = state;
    			if(Robot.armState > 0){
        			Robot.armState = state - 1;
    			}else{
    				Robot.armState = state + 1;
    			}
    			Robot.shouldCanclArmMP = true;
    			if((state == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT || state == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) && Robot.arm.getInfraredSensor()){
    	    		Robot.arm.bottomMotor.configMotionAcceleration(525, Robot.timeoutMs);
    	    	}else{
    	    		Robot.arm.bottomMotor.configMotionAcceleration(650, Robot.timeoutMs);

    	    	}
    			Robot.isArmPidRunning = true;
    			if(doneTime.get() == 0){
        			doneTime.start();
        			SmartDashboard.putBoolean("did get here", true);

        		}
    		}
    		firstTime = true;
    	
    		Robot.arm.bottomMotor.set(ControlMode.MotionMagic, value);    	Robot.arm.setArmBrake(true);
        	Robot.arm.setArmBrake(false);

    	}
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

    		if(Math.abs(Robot.arm.getArmRaw()-value) < ArmSubsystem.ArmConstants.RAW_ABS_TOL){
        		return true;
        	}
//        	else if(Math.abs(Robot.arm.getArmAngle()) > Math.abs(value)){
//        		return true;
//        	}
        	else if(Robot.arm.getArmHalZeroBack() && state == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR){
        		return true;
        	}else if(doneTime.get()>1.25){
        		return true;
        	}else if(Robot.arm.getArmHalZeroFront() && state == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT){
        		return true;
        	}
        	return false;

    }

    // Called once after isFinished returns true
    protected void end() {
    	doneTime.stop();
    	doneTime.reset();
    	if(Robot.armState > 0){
    	
    			Robot.armState++;
    
    	}else{
    			Robot.armState--;
    	}
    	if(value == ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT){
    		Robot.armState = ArmSubsystem.ArmStateConstants.INTAKE_FRONT;
    	}else if(value == ArmSubsystem.ArmEncoderConstants.INTAKE_REAR){
    		Robot.armState = ArmSubsystem.ArmStateConstants.INTAKE_REAR;

    	}
		Robot.shouldCanclArmMP = false;

    	SmartDashboard.putBoolean("ran is xd", true);
		Robot.isArmPidRunning = false;
    	Robot.arm.setArmBrake(true);
    	Robot.intake.setIntakeMotors(0, 0);
    	if(Robot.armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT   || Robot.armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR){
        	Robot.arm.bottomMotor.set(ControlMode.PercentOutput, 0);

    	}
    	Robot.isRunnigWithFlip = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
