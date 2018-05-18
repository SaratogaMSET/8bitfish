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
public class LeftMPSwitch extends CommandGroup {

    public LeftMPSwitch() {
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT,Robot.armState,false));
        addSequential(new MotionProfileDrive(false));
        addSequential(new DriveBackForTime(0.8,0.5));
        addSequential(new RunIntakeForTime(0.3,false,0.75));
        addSequential(new DrivetrainMotionProfileIn(-65));
        addParallel(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT));
        addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT,Robot.armState,false));
        addParallel(new SetIntakePistons(true,false));
        addSequential(new GyroPID(70));
    	addParallel(new RunIntakeWheels(0.3));
    	addSequential(new DrivetrainMotionProfileIn(22.5));
    	addSequential(new SetIntakePistons(false,true));
    	addParallel(new RunIntakeWheels(1.0));
    	addSequential(new WaitTime(0.5));
    	addSequential(new RunIntakeWheels(0));
    	addParallel(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT,Robot.armState,false));
    	addSequential(new DrivetrainMotionProfileIn(-67.5));
    	addSequential(new GyroPID(-70));
    	addSequential(new DriveBackForTime(0.5,0.5));
        addSequential(new RunIntakeForTime(0.35,false,1));
        
    }
}
