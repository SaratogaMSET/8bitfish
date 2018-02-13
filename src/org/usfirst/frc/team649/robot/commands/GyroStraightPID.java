package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroStraightPID extends Command {
	
	PIDController GyroController;
	PIDController DriveController;
	double distance;
	boolean isFinished;
	
    public GyroStraightPID(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.distance = distance;
    	
    	GyroController = Robot.gyro.getPIDController();
    	DriveController = Robot.drive.getPIDController();
    	
    	isFinished = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.gyro.setDrivingStraight(true);
    	Robot.drivePIDRunning = true;
    	GyroController.enable();
    	DriveController.enable();
    	
    	GyroController.setSetpoint(0);
    	DriveController.setSetpoint(distance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double left = 0;
    	double right = 0;
    	double drivePower = Robot.drive.getDrivePIDOutput();
    	double gyroPower = Robot.gyro.getGyroPIDOutput();
    	
    	if ( gyroPower > 0) {
			left = drivePower;
			right = drivePower - gyroPower;
		} else if (gyroPower < 0) {
			right = drivePower + gyroPower;
			left = drivePower;
		} else {
			right = drivePower;
			left = drivePower;
		}
    	
    	double max = Math.max(right, Math.max(left, 1));
    	
    	right /= max;
    	left /= max;
    	
		Robot.drive.rawDrive(left, right);
		
		if (DriveController.onTarget()) {
			isFinished = true;
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivePIDRunning = false;
    	GyroController.disable();
    	DriveController.disable();
    	
    	Robot.drive.rawDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
