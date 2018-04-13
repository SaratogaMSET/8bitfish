package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfileDelayed;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.liftCommands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DownAndFlipWhenPossible2ndIntakeRear extends CommandGroup {

    public DownAndFlipWhenPossible2ndIntakeRear() {
    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT,Robot.armState,false));
    	addParallel(new ArmMotionProfileDelayed(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR,Robot.armState));
        addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE,Robot.liftState,0));
    }
}
