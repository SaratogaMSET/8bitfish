package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GyroZeroDegree extends Command {
	
	double angle;
	Timer time;
	Timer timeout;
	boolean isFinished;
	boolean isTimeout;
	String actuallyFinished;
	double gyroFinal;
	PIDController drivePID;
	
    public GyroZeroDegree() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	angle = 0;
    	drivePID = Robot.gyro.getPIDController();
    	time = new Timer();
    	timeout = new Timer();
    	isFinished = false;
    	isTimeout = false;
    	actuallyFinished = "false";
    	gyroFinal = 0;
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	drivePID.enable();
    	Robot.drivePIDRunning = true;
    	double setpoint =  angle;
    	
    	
    	drivePID.setAbsoluteTolerance(GyroSubsystem.GyroPIDConstants.GYRO_ABS_TOLERANCE);
    	
    	drivePID.setSetpoint(setpoint);
    	SmartDashboard.putNumber("Setpoint", setpoint);
    	time.start();
    	timeout.start();
    	//drivePIDRight.setSetpoint(setpoint);
    	SmartDashboard.putString("Current Command", getName());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(((Math.abs(Robot.gyro.getGyroAngle() - angle))<GyroSubsystem.GyroPIDConstants.GYRO_ABS_TOLERANCE) && time.get() == 0){
    		time.start();
    	}else if(time.get() > 0.01 && !((Math.abs(Robot.gyro.getGyroAngle() - angle))<GyroSubsystem.GyroPIDConstants.GYRO_ABS_TOLERANCE)){
    		time.stop();
    		time.reset();
    	}else if(time.get()> 0.1){
    		SmartDashboard.putNumber("Gyro Final", Robot.gyro.getGyroAngle());
    		return true;
    	}
    	if(((Math.abs(Robot.gyro.getGyroAngle() - angle))<GyroSubsystem.GyroPIDConstants.GYRO_ABS_TOLERANCE)) {
    		SmartDashboard.putNumber("Gyro Final", Robot.gyro.getGyroAngle());
    		return true;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivePIDRunning = false;
    	drivePID.disable();
    	SmartDashboard.putBoolean("Timeout", isTimeout);
    	SmartDashboard.putBoolean("End", true);
    	SmartDashboard.putBoolean("pid done", true);
    	Robot.drive.rawDrive(0, 0);
//    	Robot.gyro.resetGyro();
    	Robot.gyro.setDrivingStraight(false);
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
