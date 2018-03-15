package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ZeroArmRoutine extends Command {

    public ZeroArmRoutine() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.arm.setArmBrake(false);
    	Robot.arm.setArm(-0.2);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.arm.getArmHalZeroBack()||Robot.arm.getArmHalZeroFront();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.arm.setArmBrake(true);
    	Robot.arm.setArm(0);
    	Robot.isZero = true;
    	Robot.armIsFront = true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
