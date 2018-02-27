package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoTestCommand extends Command {
	
	Timer time;
    public AutoTestCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	time = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	time.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.oi.operator.getButton4() && time.get()>0.2) {
    		Robot.autoTest.incrementTuningVarValue(true);
    	}
    	if(Robot.oi.operator.getButton6() && time.get()>0.2) {
    		Robot.autoTest.incrementTuningVarValue(false);
    	}
    	if(Robot.oi.operator.getButton5() && time.get()>0.2) {
    		Robot.autoTest.changeTuningIncrement(true);
    	}
    	if(Robot.oi.operator.getButton7() && time.get()>0.2) {
    		Robot.autoTest.changeTuningIncrement(false);
    	}
    	if(Robot.oi.operator.getButton2() && time.get()>0.2) {
    		Robot.autoTest.changeCurrentlyTuningProgram(true);
    	}
    	if(Robot.oi.operator.getButton3() && time.get()>0.2) {
    		Robot.autoTest.changeCurrentlyTuningProgram(true);
    	}
    	if(Robot.oi.operator.getButton8() && time.get()>0.2) {
    		Robot.autoTest.changeCurrentlyTuningSegment(true);
    	}
    	if(Robot.oi.operator.getButton9() && time.get()>0.2) {
    		Robot.autoTest.changeCurrentlyTuningSegment(false);
    	}
    	if(Robot.oi.operatorJoystick.getRawButton(2) && time.get()>0.2) {
    		Robot.autoTest.startProgram();
    	}
    	Robot.autoTest.SmartDashboard();
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
