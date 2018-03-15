package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetMotionMagicParameter extends Command {
	
	int velocity;
	int acceleration;
	double k_p;
	double k_i;
	double k_d;
    public SetMotionMagicParameter(int velocity, int acceleration, double k_p, double k_i, double k_d) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.velocity = velocity;
    	this.acceleration = acceleration;
    	this.k_p = k_p;
    	this.k_d = k_d;
    	this.k_i = k_i;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	Robot.drive.setOutputRange(-value, value);
    	for(int i = 0; i < 4; i ++) {
    		Robot.drive.motors[i].configMotionCruiseVelocity(velocity, Robot.timeoutMs);
    		Robot.drive.motors[i].configMotionAcceleration(acceleration, Robot.timeoutMs);
    		Robot.drive.motors[i].config_kP(0, k_p, Robot.timeoutMs);
    		Robot.drive.motors[i].config_kP(0, k_i, Robot.timeoutMs);
    		Robot.drive.motors[i].config_kP(0, k_d, Robot.timeoutMs);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
