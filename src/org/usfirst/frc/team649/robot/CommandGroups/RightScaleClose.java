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
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.WaitTime;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightScaleClose extends CommandGroup {

    public RightScaleClose() {
    	addSequential(new ChangeRobotLiftState(9));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState));
    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));

    	addSequential(new MotionProfileDrive());
    	
    	//
////    	addSequential(new SwitchMPModes(Robot.modifierRightScaleSingle2.getLeftTrajectory(),Robot.modifierRightScaleSingle2.getLeftTrajectory()));
    	addParallel(new DrivetrainPIDCommand(-15));
    	addParallel(new ChangeRobotLiftState(1));
    	addSequential(new RunIntakeForTime(0.5,false));
    	

    	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,Robot.liftState,0.1));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR));
    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR,Robot.armState));
    	addSequential(new SetIntakePistons(true,false));
    	addSequential(new DrivetrainPIDCommand(-20));
    	addParallel(new SetIntakePistons(false,false));
    	addSequential(new RunIntakeWheels(0.6));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT));
    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT,Robot.armState));
    	addParallel(new ChangeRobotLiftState(9));
    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
    	addParallel(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState));
    	addSequential(new DrivetrainPIDCommand(35));
    	addSequential(new RunIntakeForTime(1,false));
    	////    	addSequential(new GyroZeroDegree());
    }
}
