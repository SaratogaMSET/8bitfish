package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPIDBack;
import org.usfirst.frc.team649.robot.commands.intake.SetIntakePistons;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftBack extends CommandGroup {

    public LeftBack() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	addSequential(new GyroPIDBack(70));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR));
    	addSequential(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, Robot.armState, false));
    	addSequential(new SetIntakePistons(true, false));
    	addSequential(new MotionProfileDriveInverted(true, true));
    }
}
