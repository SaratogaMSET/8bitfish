package org.usfirst.frc.team649.robot.commands.intake;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntakePistons extends Command {
	
	boolean isOpen;
	boolean isClamp;
    public SetIntakePistons(boolean isOpen, boolean isClamp) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.isOpen = isOpen;
    	this.isClamp = isClamp;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	if(isOpen){
    		Robot.isOpen = true;
    		Robot.intake.setIntakePiston60(false);
    		Robot.intake.setIntakePiston30(true);
    		
    	}else{
    		Robot.isOpen = false;
    		if(isClamp){
    			Robot.intake.setIntakePiston60(true);
        		Robot.intake.setIntakePiston30(false);

    		}else{
    			Robot.intake.setIntakePiston60(true);

    			Robot.intake.setIntakePiston30(true);
    		}
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
