package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 Autonomous 4:
 *Center Autonomous delivers to Switch on right side
 *****Drive forward, turn and drive diagonally, 
 *****then turn to straight again and drive forward a 
 *****little and drop off block
 *Positions: Center
 */
public class CenterTwoSwitchLeft extends CommandGroup {

    public CenterTwoSwitchLeft() {
    	addSequential(new ZeroArmRoutine());
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterLeftSwitch.FIRST_DRIVE)); // drive straight
    	addSequential(new GyroPID(AutoTest.CenterLeftSwitch.FIRST_ANGLE_TURN)); // turn ~45 degrees
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, Robot.armState, false));
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterLeftSwitch.SECOND_DRIVE)); // drive straight diagonally    	addSequential(new GyroPID(AutoTest.CenterLeftSwitch.SECOND_ANGLE_TURN));// turn back to straight
       	addSequential(new RunIntakeForTime(1, false, 1));
    	addSequential(new DrivetrainMotionProfileIn(-10));
    	addSequential(new ZeroArmRoutine());
    	addSequential(new DrivetrainMotionProfileIn(10));
    	addSequential(new RunIntakeForTime(1, true, 1));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, Robot.armState, false));
    	addSequential(new RunIntakeForTime(1, false, 1));
    	addSequential(new DrivetrainMotionProfileIn(-10));
    	addSequential(new ZeroArmRoutine());
    }
}
