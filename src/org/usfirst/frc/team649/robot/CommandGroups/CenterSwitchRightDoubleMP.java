package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.autonomous.AutoTest;
import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterSwitchRightDoubleMP extends CommandGroup  {
	
	public CenterSwitchRightDoubleMP() {
        addSequential(new MotionProfileDrive(false));
    }
}
