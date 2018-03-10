package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.RunIntakeForTime;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightScaleClose extends CommandGroup {

    public RightScaleClose() {
    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState));
    	addSequential(new MotionProfileDrive());
//    	addSequential(new SwitchMPModes(Robot.modifierRightScaleSingle2.getLeftTrajectory(),Robot.modifierRightScaleSingle2.getLeftTrajectory()));
    	addSequential(new RunIntakeForTime(1,false));
    }
}
