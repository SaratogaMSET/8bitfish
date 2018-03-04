package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DownLiftUntilSafe extends Command {
	double idealPos;
	Timer doneTime;
	Timer timeout;
	int state;
	boolean furtherDown;
    public DownLiftUntilSafe() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timeout = new Timer();
    	doneTime = new Timer();
    	timeout.start();
    	furtherDown = false;
    	Robot.lift.mainLiftMotor.configMotionCruiseVelocity(4200, Robot.timeoutMs);
		Robot.lift.mainLiftMotor.configMotionAcceleration(4000, Robot.timeoutMs); // 400 actual
		Robot.lift.mainLiftMotor.selectProfileSlot(0, 0);
		Robot.lift.mainLiftMotor.config_kF(0, 0.3197, Robot.timeoutMs);
		Robot.lift.mainLiftMotor.config_kP(0, 4, Robot.timeoutMs);
		Robot.lift.mainLiftMotor.config_kI(0, 0, Robot.timeoutMs);
		Robot.lift.mainLiftMotor.config_kD(0, 0, Robot.timeoutMs);
    	idealPos = Robot.lift.getRawLift() - Robot.lidarValue*LiftSubsystem.LiftConstants.unitsPerCmSecond - (Robot.lift.getCarriageHeight()-LiftSubsystem.LiftConstants.flipUpperPos)*LiftSubsystem.LiftConstants.unitsPerCmCarriage;
		SmartDashboard.putNumber("ideal pos", idealPos);

    	if(idealPos<0){
    		idealPos = 0;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(furtherDown){
    		Robot.lift.setLift(0.4);
    	}else{
        	Robot.lift.mainLiftMotor.set(ControlMode.MotionMagic, idealPos);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.lift.getCarriageHeight()<LiftSubsystem.LiftConstants.flipUpperPos){
    		return true;
    	}else if(!furtherDown){
    		if(doneTime.get() > 0.15){
    			furtherDown = true;
        	}else if(!(doneTime.get() == 0) && Math.abs(idealPos-Robot.lift.mainLiftMotor.getSelectedSensorPosition(0)) > 100){
        		doneTime.stop();
        		doneTime.reset();
        	}else if(Math.abs(Robot.lift.getRawLift() - idealPos)< 100 && doneTime.get() == 0){
        		doneTime.start();
        	} else if (timeout.get() > 5) {
        		furtherDown = true;
        	}else if(Math.abs(Robot.lift.getRawLift()-idealPos)<LiftSubsystem.LiftConstants.absTol){
        		furtherDown = true;
        	}
    	}
		return false;

     	
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.lift.mainLiftMotor.set(ControlMode.PercentOutput, 0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
