package org.usfirst.frc.team649.autonomous.worlds;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeRear;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.SwitchMPModes;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
<<<<<<< HEAD:src/org/usfirst/frc/team649/robot/CommandGroups/RightScaleSingleMP.java
import org.usfirst.frc.team649.robot.commands.drivetrain.DriveBackForTime;
=======
import org.usfirst.frc.team649.robot.commands.arm.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.commands.drivetrain.DriveBackForTime;
import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
>>>>>>> master:src/org/usfirst/frc/team649/autonomous/worlds/RightScaleSingleMP.java
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.liftCommands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.liftCommands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous Path:
 * Description: Drives forward and drops off cube in the scale
 * Position: Right*/

// AUTO TESTED

public class RightScaleSingleMP extends CommandGroup {
	public RightScaleSingleMP() {
		addSequential(new ChangeRobotLiftState(9));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState,false));
    	addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
    	addSequential(new MotionProfileDrive(false));    	
       	addParallel(new ChangeRobotLiftState(1));
    	addSequential(new RunIntakeForTime(0.5, false, 1));
    	addSequential(new DriveBackForTime(-0.3, 0.5)); // move back from scale for safety
<<<<<<< HEAD:src/org/usfirst/frc/team649/robot/CommandGroups/RightScaleSingleMP.java
    }
=======
    	//TODO: Fix move back, left scale single MP works, so change based on that
	}
>>>>>>> master:src/org/usfirst/frc/team649/autonomous/worlds/RightScaleSingleMP.java
}
