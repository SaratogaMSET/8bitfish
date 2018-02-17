package org.usfirst.frc.team649.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDCurves extends Command {

	double x;
	double y;
	double angle;
	
    public PIDCurves(double x, double y) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.x = x;
    	this.y = y;
    	this.angle = Math.toDegrees(Math.tan(y/x));
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
