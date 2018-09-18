package org.usfirst.frc.team649.robot.commands.liftCommands;

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
	Timer stallTime;
	double prevEncoder;
	
	boolean wasCancl;
	
    public LiftMotionProfile(int encoderValue, int state, double time) {
    	value = encoderValue;
        this.state = state;
        waitTime = time;
        SmartDashboard.putNumber("Val", value);
        SmartDashboard.putBoolean("started", true);
        stallTime = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.compressor.stop();
    	
    	SmartDashboard.putBoolean("ran is fin", false);
    	
    	state = Robot.liftState;
    	
    	startTime = new Timer();
    	timeout = new Timer();
    	doneTime = new Timer();
    	startTime.start();
    	
    	wasCancl = false;
    	
    	if(value > Robot.lift.getRawLift()){
    		Robot.lift.mainLiftMotor.configMotionCruiseVelocity(4500, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.configMotionAcceleration(5300, Robot.timeoutMs); // 400 actual
			Robot.lift.mainLiftMotor.selectProfileSlot(0, 0);
			Robot.lift.mainLiftMotor.config_kF(0, 0.432, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.config_kP(0, 6.3, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.config_kD(0, 0.05, Robot.timeoutMs);
    	}else{

    		Robot.lift.mainLiftMotor.configMotionCruiseVelocity(5200, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.configMotionAcceleration(7000, Robot.timeoutMs); // 400 actual
			Robot.lift.mainLiftMotor.selectProfileSlot(0, 0);
			Robot.lift.mainLiftMotor.config_kF(0, 0.411, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.config_kP(0, 1.5, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
			Robot.lift.mainLiftMotor.config_kD(0, 0.01 , Robot.timeoutMs);
    	}
//    	timeout.start();
    	System.out.println("LiftMotionProfile" + state);    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putBoolean("running", true);
    	if(Robot.isZero && startTime.get() > waitTime){
    		if(timeout.get() == 0){
    			timeout.start();
    			stallTime.start();
    			prevEncoder = Robot.lift.getRawLift();
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
        	} else if (timeout.get() > 2.5) {
        		return true;
        	}
        	else if(state != Robot.liftState ){
        		wasCancl = true;
        		return true; 
        	}else if(Math.abs(Robot.lift.getRawLift()-value)<LiftSubsystem.LiftConstants.absTol){
        		return true;
        	}
        	//else if(Robot.lift.getLiftState() == LiftSubsystem.LiftStateConstants.LO)
        	return false;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.compressor.start();
    	if(!wasCancl){
    		if(state == LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP){
        		Robot.liftState--;
        	}else{
        		Robot.liftState++;
        	}
    	}
    	
//    	Robot.lift.mainLiftMotor.set(ControlMode.PercentOutput, 0);
    	SmartDashboard.putBoolean("ran is fin", true);
    	SmartDashboard.putBoolean("did cancl", wasCancl);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
