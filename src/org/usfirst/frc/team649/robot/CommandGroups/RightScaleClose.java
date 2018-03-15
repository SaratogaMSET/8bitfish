package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroZeroDegree;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.WaitTime;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightScaleClose extends CommandGroup {

    public RightScaleClose() {
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addParallel(new MotionProfileDrive());
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState));

    	addSequential(new WaitTime(1));
    	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState));
//    	addSequential(new SwitchMPModes(Robot.modifierRightScaleSingle2.getLeftTrajectory(),Robot.modifierRightScaleSingle2.getLeftTrajectory()));
    	addSequential(new RunIntakeForTime(1,false));
    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE));
    	addParallel(new DrivetrainPIDCommand(-35));
    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,Robot.liftState));
//    	addSequential(new GyroZeroDegree());
    }
}
