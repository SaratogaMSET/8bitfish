package org.usfirst.frc.team649.robot.commands.arm;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class MoveArmCommand extends Command {

	int targetPosition;
	boolean isIRTripped;
	boolean isFinished;
	boolean customUp;

	public MoveArmCommand(int targetPosition, boolean customUp) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.targetPosition = targetPosition;
		this.customUp = customUp;
		isIRTripped = false;
		isFinished = false;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		isIRTripped = Robot.arm.getInfraredSensor();
		SmartDashboard.putNumber("ArmMove", targetPosition);
		chooseArmCommand();
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

	private void chooseArmCommand() {
		switch (targetPosition) {
		case ArmSubsystem.ArmStateConstants.INTAKE_FRONT:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT
					&& Robot.armState != ArmSubsystem.ArmStateConstants.INTAKE_FRONT) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Intake Front", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.INTAKE_REAR:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR
					&& Robot.armState != ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Intake Back", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.STORE_FRONT:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT
					&& Robot.armState != ArmSubsystem.ArmStateConstants.STORE_FRONT) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Store Front", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.STORE_REAR:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR
					&& Robot.armState != ArmSubsystem.ArmStateConstants.STORE_REAR) {

				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Store Back", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT
					&& Robot.armState != ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Exchange Front", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.EXCHANGE_REAR:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR
					&& Robot.armState != ArmSubsystem.ArmStateConstants.EXCHANGE_REAR) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Exchange Back", true);
			isFinished = true;
			break;
			
		case ArmSubsystem.ArmStateConstants.MID_DROP_FRONT:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
					&& Robot.armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Mid Drop Front", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.MID_DROP_REAR:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
					&& Robot.armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("Mid Drop Back", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
					&& Robot.armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("High Drop Front", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR:
			if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
					&& Robot.armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
				Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
				new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, Robot.armState, isIRTripped).start();
			}
			SmartDashboard.putBoolean("High Drop Back", true);
			isFinished = true;
			break;

		case ArmSubsystem.ArmStateConstants.CUSTOM:
			if (customUp) {
				if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
						&& Robot.arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ < 0) {
					Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
					if (Robot.arm.getArmRaw()
							+ ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID) {
						Robot.armIsFront = true;
					} else {
						Robot.armIsFront = false;
					}
					Robot.customArmPos = Robot.arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ;
					new ArmMotionProfile(Robot.customArmPos, Robot.armState, isIRTripped).start();
				}
			} else if (!customUp) {
				if (Robot.armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN && Robot.arm.getArmRaw()
						- ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.INTAKE_REAR) {
					Robot.armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
					if (Robot.arm.getArmRaw()
							- ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID) {
						Robot.armIsFront = true;
					} else {
						Robot.armIsFront = false;
					}
					Robot.customArmPos = Robot.arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ;
					new ArmMotionProfile(Robot.customArmPos, Robot.armState, isIRTripped).start();
				}
			}
			isFinished = true;
			break;
		default:
			isFinished = true;
			break;
		}
	}
}
