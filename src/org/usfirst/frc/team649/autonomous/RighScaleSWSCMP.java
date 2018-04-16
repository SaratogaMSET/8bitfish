package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeRear;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.drivetrain.DriveBackForTime;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.intake.SetIntakePistons;
import org.usfirst.frc.team649.robot.commands.liftCommands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RighScaleSWSCMP extends CommandGroup {

    public RighScaleSWSCMP() {
//    	addSequential(new ChangeRobotLiftState(9));
//    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
//    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,false));
//    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
    	addSequential(new MotionProfileDrive(false)); 
    	addParallel(new RunIntakeForTime(0.5,false,1));
    	addParallel(new DriveBackForTime(-0.3,0.9));
//    	addSequential(new SwitchMPModes(Robot.modifierSideBack));
    	addSequential(new DownAndFlipWhenPossibleIntakeRear());
    	addSequential(new SetIntakePistons(true,false));
    	addParallel(new RunIntakeWheels(0.5));
    	addSequential(new DriveBackForTime(-0.3,1));
    	addSequential(new SetIntakePistons(false,false));
    	addSequential(new Delay(0.75));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR));
    	addParallel(new DriveBackForTime(0.3,0.5));
    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR,Robot.armState,true));
    	addSequential(new DriveBackForTime(-0.3,0.5));
    	addSequential(new RunIntakeForTime(1,false,1));
//    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
//    	addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE));
//    	addParallel(new DrivetrainMotionProfileIn(55));
//    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,true));
//
//    	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,0));
//    	addSequential(new RunIntakeForTime(0.5,false));
    	//    	addSequential(new RunIntakeForTime(1,false));
//    	addSequential(new SwitchMPModes(Robot.modifierSideBack));
//    	addSequential(new MotionProfileDrive(false));
    	
//    	addParallel(new DrivetrainPIDCommand(-15));
//    	addParallel(new ChangeRobotLiftState(1));
//    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR));
//    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR,Robot.armState));
//    	addSequential(new DrivetrainPIDCommand(-20));
//    	addSequential(new RunIntakeWheels(0.6));
//    	addSequential(new SetIntakePistons(false,false));    	
//    	addParallel(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR));
//    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR,Robot.armState));
//    	addSequential(new DrivetrainPIDCommand(-10));
//    	addSequential(new RunIntakeForTime(1,false));
    }
}
