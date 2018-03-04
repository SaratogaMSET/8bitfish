package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunIntakeForTime extends Command {

	double time;
	boolean isIntake;
	Timer timeOf;
	
    public RunIntakeForTime(double time,boolean isIntake) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.time = time;
    	this.isIntake = isIntake;
    	timeOf = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(isIntake){
    		Robot.intake.setIntakeMotors(-1, -1);
    	}else{
    		Robot.intake.setIntakeMotors(1, 1);
    	}
    	timeOf.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(timeOf.get() > time){
    		return true;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	timeOf.stop();
    	timeOf.reset();
    	Robot.intake.setIntakeMotors(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
