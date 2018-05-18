package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeRear;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.SwitchMPModes;
import org.usfirst.frc.team649.robot.commands.WaitTime;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.drivetrain.DriveBackForTime;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPIDBack;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.intake.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.liftCommands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/** Left Scale Motion Profile
 * Description: Left side scale autonomous, motion profiled
 * */

public class LeftScaleMP extends CommandGroup {
	public LeftScaleMP() {
		addSequential(new ChangeRobotLiftState(9));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,false));
    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
    	addSequential(new MotionProfileDrive(false));    	
    	addParallel(new ChangeRobotLiftState(1));
    	addSequential(new RunIntakeForTime(0.6, false, 0.35));
    	addParallel(new GyroPIDBack(57));
    	addSequential(new DownAndFlipWhenPossibleIntakeRear());

    	addParallel(new SetIntakePistons(true,false));
     	addParallel(new RunIntakeWheels(0.5));
     	addSequential(new DriveBackForTime(-0.55,0.7));
     	addSequential(new SetIntakePistons(false,true));
     	addSequential(new WaitTime(0.85));
     	addParallel(new RunIntakeWheels(0));
     	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE));
    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,true));
    	addSequential(new GyroPID(45));
//    	addParallel(new DrivetrainMotionProfileIn(10));
//    	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,0.5));
//    	addSequential(new RunIntakeForTime(0.5, false, 0.5));
	}
}
