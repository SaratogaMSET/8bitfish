package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class SwitchMPModes extends Command {
	TankModifier tank;
    public SwitchMPModes(TankModifier path) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	tank = path;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.left.reset();
    	Robot.right.reset();
 
    	Robot.left = new EncoderFollower(tank.getLeftTrajectory());
    	Robot.right = new EncoderFollower(tank.getRightTrajectory());
    	Robot.left.configureEncoder(0, 4096 * 2, 0.127);
		Robot.right.configureEncoder(0, 4096 * 2, 0.127);
		Robot.left.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);
		Robot.right.configurePIDVA(2, 0.0, 0, 1 / 4.5, 0);

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
    	Robot.left.reset();
    	Robot.right.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
