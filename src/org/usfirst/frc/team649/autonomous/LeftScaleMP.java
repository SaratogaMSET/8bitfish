package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeRear;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.SwitchMPModes;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
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
    	addSequential(new RunIntakeForTime(0.5,false));
    	//addSequential(new SwitchMPModes(Robot.modifierSideBack));
    	//addSequential(new MotionProfileDrive(false));
    	addParallel(new DownAndFlipWhenPossibleIntakeRear());
    	addSequential(new SwitchMPModes(Robot.modifierSideBack));
    	addSequential(new MotionProfileDrive(true));
	}
}
