package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous 5:
 * Just drive straight
 * Position: any
 */
public class DriveStraight extends CommandGroup {

    public DriveStraight() {
    	addSequential(new ZeroArmRoutine());
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, Robot.armState,false));
    	addSequential(new DrivetrainMotionProfileIn(100)); // Drive Straight
    }
}
