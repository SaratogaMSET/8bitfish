package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfileDelayed;
import org.usfirst.frc.team649.robot.commands.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DownAndFlipWhenPossible2ndIntakeFront extends CommandGroup {

    public DownAndFlipWhenPossible2ndIntakeFront() {
    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR,Robot.armState));
    	addParallel(new ArmMotionProfileDelayed(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT,Robot.armState));
        addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE,Robot.liftState,0));
    }
}
