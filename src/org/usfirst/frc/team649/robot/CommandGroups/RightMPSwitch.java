package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.WaitTime;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.drivetrain.DriveBackForTime;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.intake.SetIntakePistons;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightMPSwitch extends CommandGroup {

    public RightMPSwitch() {
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT,Robot.armState,false));
        addSequential(new MotionProfileDrive(false));
        addSequential(new RunIntakeForTime(0.5,false,1));
        addSequential(new DrivetrainMotionProfileIn(-63.5));
        addParallel(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT));
        addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT,Robot.armState,false));
        addParallel(new SetIntakePistons(true,false));
        addSequential(new GyroPID(-75));
    	addParallel(new RunIntakeWheels(0.3));
    	addSequential(new DrivetrainMotionProfileIn(37.5));
    	addSequential(new SetIntakePistons(false,true));
    	addParallel(new RunIntakeWheels(1.0));
    	addSequential(new WaitTime(1.5));
    	addSequential(new RunIntakeWheels(0));
    	addParallel(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT,Robot.armState,false));
    	addSequential(new DrivetrainMotionProfileIn(-62.5));
    	addSequential(new GyroPID(85));
//    	addSequential(new DrivetrainMotionProfileIn(22));

    	addSequential(new DriveBackForTime(0.5,1));
        addSequential(new RunIntakeForTime(0.5,false,1));
//      addSequential(new DrivetrainPIDCommand(-25));
    }
}
