package org.usfirst.frc.team649.robot.commands.intake;

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
	double power;
	
    public RunIntakeForTime(double time,boolean isIntake, double power) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.time = time;
    	this.isIntake = isIntake;
    	timeOf = new Timer();
    	this.power = power;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(!Robot.isMPRunning){
    		if(timeOf.get() == 0){
    			if(isIntake){
            		Robot.intake.setIntakeMotors(-power, -power);
            	}else{
            		Robot.intake.setIntakeMotors(power, power);
            	}
            	timeOf.start();
    		}
    		
    	}
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
