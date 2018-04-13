package org.usfirst.frc.team649.robot.commands.arm;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FlipArmCommand extends Command {

	int currentState;
	boolean isIRTriggered;
    public FlipArmCommand(int currentState) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.currentState = currentState;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	isIRTriggered = Robot.arm.getInfraredSensor();
    	flipTheArm();
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
    
    private void flipTheArm() {
		if (currentState == ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
				|| currentState == ArmSubsystem.ArmStateConstants.CUSTOM) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - Robot.customArmPos, currentState,
					isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
				|| currentState == ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.STORE_FRONT
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.EXCHANGE_REAR
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN
				|| currentState == ArmSubsystem.ArmStateConstants.CUSTOM) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - Robot.customArmPos, currentState,
					isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
				|| currentState == ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, currentState, isIRTriggered).start();
		} else if (currentState == ArmSubsystem.ArmStateConstants.MID_DROP_REAR
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, currentState, isIRTriggered).start();

		} else if (currentState == ArmSubsystem.ArmStateConstants.STORE_REAR
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, currentState, isIRTriggered).start();

		} else if (currentState == ArmSubsystem.ArmStateConstants.INTAKE_REAR
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, currentState, isIRTriggered).start();
			
		} else if (currentState == ArmSubsystem.ArmStateConstants.SWITCH_REAR
				|| currentState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR) {
			currentState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
			Robot.armState = currentState;
			new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, currentState, isIRTriggered).start();
		}

    }
}
