package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.liftCommands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous Path:
 * Description: Drives forward and drops off cube in the scale, then backs away 
 * Position: Right*/

public class RightScaleNoTurn extends CommandGroup {
	public RightScaleNoTurn() {
		addSequential(new ZeroArmRoutine());
		addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleNoTurnVal.DRIVE_TO_SCALE));
		/*addSequential(new GyroPID(AutoTest.RightScaleNoTurnVal.TURN_ANGLE_WHILE_DRIVING));
		// Copied from RightScale, puts lift at high position and deposits cube
		addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
        addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,false));
    	//addSequential(new RunIntakeForTime(1, false));
    	addSequential(new DrivetrainMotionProfileIn(-10));// deploy
    	//TODO Move backwards
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleNoTurnVal.DRIVE_BACKWARDS));
    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE));*/
	}
}
