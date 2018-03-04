package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

/**
 *
 */
public class MotionProfileDrive extends Command {

    public MotionProfileDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("isMPFin", false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double l = Robot.left.calculate(Robot.drive.motors[0].getSelectedSensorPosition(0));
		double r = Robot.right.calculate(Robot.drive.motors[2].getSelectedSensorPosition(0));

		double gyro_heading = -Robot.gyro.getGyroAngle();
		double desired_heading = Pathfinder.r2d(Robot.left.getHeading());  
		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		//1.1 for middle left
		double turn = 0.8 * (-1.0/80.0) * angleDifference;
		SmartDashboard.putNumber("turn", turn);
		Robot.drive.motors[0].set(ControlMode.PercentOutput, l+turn);
		Robot.drive.motors[2].set(ControlMode.PercentOutput, r-turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.left.isFinished()&&Robot.right.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putBoolean("isMPFin", true);

    	Robot.drive.rawDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}