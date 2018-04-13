package org.usfirst.frc.team649.robot.commands.liftCommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class MoveLiftCommand extends Command {

	int targetPosition;
	boolean isFinished;
	boolean customUp;

	public MoveLiftCommand(int targetPosition, boolean customUp) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.targetPosition = targetPosition;
		this.customUp = customUp;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		isFinished = false;
		chooseLiftCommand();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}

	private void chooseLiftCommand() {
		switch (targetPosition) {
		case LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE:
			if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
					&& Robot.liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
				Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, Robot.liftState, 0).start();
			}
			isFinished = true;
			break;

		case LiftSubsystem.LiftStateConstants.INTAKE_2:
			if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2
					&& Robot.liftState != LiftSubsystem.LiftStateConstants.INTAKE_2) {
				Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE, Robot.liftState, 0).start();
			}
			isFinished = true;
			break;

		case LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE:
			if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE
					&& Robot.liftState != LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
				Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE, Robot.liftState, 0).start();
			}
			isFinished = true;
			break;

		case LiftSubsystem.LiftStateConstants.MID_SCALE_STATE:
			if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE
					&& Robot.liftState != LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
				Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE, Robot.liftState, 0).start();
			}
			isFinished = true;
			break;

		case LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE:
			if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE
					&& Robot.liftState != LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
				Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE, Robot.liftState, 0).start();
			}
			isFinished = true;
			break;

		case LiftSubsystem.LiftStateConstants.CUSTOM_STATE:
			if (customUp) {
				if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP && Robot.lift
						.getRawLift()
						+ LiftSubsystem.LiftEncoderConstants.ADJ_DIST < LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE) {
					Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP;
					Robot.customLiftPos = (int) Robot.lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
					new LiftMotionProfile(Robot.customLiftPos, Robot.liftState, 0).start();
				}
				isFinished = true;
			} else if (!customUp) {
				if (Robot.liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN
						&& Robot.lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST > 0) {
					Robot.liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN;
					Robot.customLiftPos = (int) Robot.lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
					new LiftMotionProfile(Robot.customLiftPos, Robot.liftState, 0).start();
				}
				isFinished = true;
			}
			break;
			
		default:
			isFinished = true;
			break;
		}
	}
}
