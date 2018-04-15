package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.autonomous.AutoTest;
import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterSwitchRightDoubleMP extends CommandGroup  {
	
	public CenterSwitchRightDoubleMP() {
		addSequential(new ZeroArmRoutine());
		addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, Robot.armState,false));
        addSequential(new MotionProfileDrive(false));
    }
}
