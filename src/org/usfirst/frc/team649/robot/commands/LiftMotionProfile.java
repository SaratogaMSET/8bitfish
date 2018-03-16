package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

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
	Timer startTime;
	int state;
	double waitTime;
	
	boolean wasCancl;
	
    public LiftMotionProfile(int encoderValue, int state, double time) {
    	value = encoderValue;
        this.state = state;
        waitTime = time;
        SmartDashboard.putNumber("Val", value);
        SmartDashboard.putBoolean("started", true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	SmartDashboard.putBoolean("ran is fin", false);
//		Robot.isArmPidRunning = true;
    	state = Robot.liftState;
    	startTime = new Timer();
    	timeout = new Timer();
    	doneTime = new Timer();
    	startTime.start();
    	wasCancl = false;
//    	timeout.start();
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putBoolean("running", true);
    	if(Robot.isZero && startTime.get() > waitTime){
    		if(timeout.get() == 0){
    			timeout.start();
    			if(value > Robot.lift.getRawLift()){
    	    		Robot.lift.mainLiftMotor.configMotionCruiseVelocity(3200, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.configMotionAcceleration(3650, Robot.timeoutMs); // 400 actual
    				Robot.lift.mainLiftMotor.selectProfileSlot(0, 0);
    				Robot.lift.mainLiftMotor.config_kF(0, 0.307, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.config_kP(0, 5.5, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.config_kD(0, 0.05, Robot.timeoutMs);
    	    	}else{
    	    		Robot.lift.mainLiftMotor.configMotionCruiseVelocity(4200, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.configMotionAcceleration(4000, Robot.timeoutMs); // 400 actual
    				Robot.lift.mainLiftMotor.selectProfileSlot(0, 0);
    				Robot.lift.mainLiftMotor.config_kF(0, 0.3197, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.config_kP(0, 4, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
    				Robot.lift.mainLiftMotor.config_kD(0, 0, Robot.timeoutMs);
    	    	}
    		}
        	Robot.lift.mainLiftMotor.set(ControlMode.MotionMagic, value);
    	}
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.isZero){
    		if(doneTime.get() > 0.15){
        		return true;
        		
        	}else if(!(doneTime.get() == 0) && Math.abs(value-Robot.lift.mainLiftMotor.getSelectedSensorPosition(0)) > 100){
        		doneTime.stop();
        		doneTime.reset();
        	}else if(Math.abs(Robot.lift.getRawLift() - value)< 100 && doneTime.get() == 0){
        		doneTime.start();
        	} else if (timeout.get() > 5) {
        		return true;
        	}
        	else if(state != Robot.liftState ){
        		wasCancl = true;
        		return true; 
        	}else if(Math.abs(Robot.lift.getRawLift()-value)<LiftSubsystem.LiftConstants.absTol){
        		return true;
        	}
        	return false;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	if(!wasCancl){
    		if(state == LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP){
        		Robot.liftState--;
        	}else{
        		Robot.liftState++;
        	}
    	}
    	
    	Robot.lift.mainLiftMotor.set(ControlMode.PercentOutput, 0);
    	SmartDashboard.putBoolean("ran is fin", true);
    	SmartDashboard.putBoolean("did cancl", wasCancl);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
