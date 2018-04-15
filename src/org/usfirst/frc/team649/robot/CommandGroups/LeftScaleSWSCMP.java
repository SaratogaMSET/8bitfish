package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
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
public class LeftScaleSWSCMP extends CommandGroup {

    public LeftScaleSWSCMP() {
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
    	addSequential(new RunIntakeForTime(1,false, 1));
    }
}
