package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainMotionProfileIn extends Command {

	double setpoint;
	boolean isFinished;
	double converted;
    public DrivetrainMotionProfileIn(double setpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.setpoint = setpoint;
    	isFinished = false;

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	isFinished = false;
    	Robot.drive.changeBrakeCoast(true);
    	Robot.isDrivePIDRunning = true;
    	Robot.drive.resetEncoders();
    	converted = Robot.drive.convert(setpoint);
    	SmartDashboard.putNumber("Converted", converted);
    	SmartDashboard.putBoolean("Is Here in Execute", false);
    	
    	Robot.drive.motors[0].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
    	Robot.drive.motors[2].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
		
    	
		SmartDashboard.putNumber("Setpoint", setpoint);
		SmartDashboard.putString("Current Command", getName());
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.motors[0].set(ControlMode.MotionMagic,(int) converted );
		Robot.drive.motors[1].set(ControlMode.Follower, RobotMap.Drivetrain.MOTOR_PORTS[0]);
		Robot.drive.motors[2].set(ControlMode.MotionMagic, -(int)converted);
		Robot.drive.motors[3].set(ControlMode.Follower, RobotMap.Drivetrain.MOTOR_PORTS[2]);
//		SmartDashboard.putNumber("Setpoint", ((setpoint/(4.0*Math.PI))/ (14.0/60.0)) * 4096);
		
//		if (Math.abs(Robot.drive.motors[0].getEncPosition() - setpoint) < 2) {
//			isFinished = true;
//		}
    	SmartDashboard.putBoolean("Is Here in Execute", true);
    	SmartDashboard.putNumber("Left Talon Distance", Robot.drive.getTalonDistanceLeft());
    	
    	if (Math.abs(converted - Robot.drive.motors[0].getSelectedSensorPosition(0)) < 100) {//(Math.abs(Robot.drive.getTalonDistanceLeft() - setpoint) < 2) {
			isFinished = true;
		}
    }
    // ((setpoint/(4*Math.PI))/ (14*60)) * 2048;  
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	SmartDashboard.putBoolean("Is TalonDistance Finished?", true);
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.motors[0].set(ControlMode.PercentOutput, 0);
    	Robot.drive.motors[2].set(ControlMode.PercentOutput, 0);
    	Robot.isDrivePIDRunning = false;
    	Robot.drive.changeBrakeCoast(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
