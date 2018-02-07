package org.usfirst.frc.team649.robot.liftCommands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftPID extends Command {
	double distanceInches;
	double rawDistance;
	public PIDController liftPID;
	Timer time;
	boolean isFinished;
    public LiftPID(double distance) {
    	distanceInches = distance;
    	rawDistance = Robot.lift.getInchesToRaw(distance);
    	liftPID = Robot.lift.getPIDController();
    	time = new Timer();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
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
