package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.liftCommands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 2:
 ***** Drive straight and turn then drop off block
 *Position: Left
 */
public class LeftFarScale extends CommandGroup {

    public LeftFarScale() {
    	addSequential(new ZeroArmRoutine());
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, Robot.armState,false));
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.FIRST_DRIVE)); // drive straight
    	addSequential(new GyroPID(AutoTest.LeftScaleFarVal.FIRST_ANGLE_TURN)); // turn ~45 degrees
    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,false));
        addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.SECOND_DRIVE));// drive straight
        addSequential(new GyroPID(AutoTest.LeftScaleFarVal.SECOND_ANGLE_TURN));
//        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.THIRD_DRIVE));
//    	addSequential(new RunIntakeForTime(0.35, false, 0.5));
//        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.THIRD_DRIVE));
        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.THIRD_DRIVE));
    	addSequential(new RunIntakeForTime(1, false, 0.5));
    	addParallel(new DrivetrainMotionProfileIn(-25));// deploy
    	addSequential(new ChangeRobotLiftState(1));
    	addSequential(new RunIntakeForTime(0.5,false,1));
    	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE,Robot.liftState,0.1));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT));
    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT,Robot.armState,false));
    }
}
