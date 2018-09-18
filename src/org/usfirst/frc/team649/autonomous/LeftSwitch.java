package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.DriveBackForTime;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeWheels;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 1:
 *****Start on left side and drive forward, turn 90 degrees and drive forward to 
 *****drop off the power cube
 *Position: Left
 */
public class LeftSwitch extends CommandGroup {

    public LeftSwitch() {
    	addSequential(new ZeroArmRoutine());
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, Robot.armState,false));
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchVal.FIRST_DRIVE));// drive forward
      	addSequential(new GyroPID(AutoTest.LeftSwitchVal.FIRST_ANGLE_TURN));// turn 90 degrees
//      	addSequential(new DriveBackForTime(0.4,0.5));
      	addSequential(new DriveBackForTime(0.4,0.5));
//      	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchVal.SECOND_DRIVE)); // drive forward
    	addSequential(new RunIntakeForTime(1, false, 1));
    	addSequential(new DrivetrainMotionProfileIn(-10));
    }
}
