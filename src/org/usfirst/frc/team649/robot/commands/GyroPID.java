package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GyroPID extends Command {
	
	double angle;
	Timer time;
	Timer timeout;
	boolean isFinished;
	boolean isTimeout;
	String actuallyFinished;
	PIDController drivePID;
	
    public GyroPID(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.angle = angle;
    	drivePID = Robot.drive.getPIDController();
    	time = new Timer();
    	timeout = new Timer();
    	isFinished = false;
    	isTimeout = false;
    	actuallyFinished = "false";
    	
    	if(angle == 0) {
    		Robot.gyro.setDrivingStraight(true);
    	} else {
    		Robot.gyro.setDrivingStraight(false);
    	}
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	drivePID.enable();
    	double setpoint = Robot.gyro.getGyroAngle() + angle;
    	
    	Robot.gyro.resetGyro();
    	
    	drivePID.setSetpoint(setpoint);
    	SmartDashboard.putNumber("Setpoint", setpoint);
    	time.start();
    	timeout.start();
    	//drivePIDRight.setSetpoint(setpoint);
    	SmartDashboard.putString("Current Command", getName());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	double output = Robot.gyro.GyroPIDOutput;
    	Robot.drive.rawDrive(output, -output);
    	if (time.get() <= 0) {
    		time.start();
    	}
    	
    	if (time.get() > 6) {
    		isFinished = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	drivePID.disable();
    	SmartDashboard.putBoolean("Timeout", isTimeout);
    	SmartDashboard.putBoolean("End", true);
    	SmartDashboard.putBoolean("pid done", true);
    	Robot.drive.rawDrive(0, 0);
    	Robot.gyro.resetGyro();
    	Robot.gyro.setDrivingStraight(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
