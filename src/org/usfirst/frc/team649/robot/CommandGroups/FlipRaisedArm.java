package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.DownLiftUntilSafe;
import org.usfirst.frc.team649.robot.commands.arm.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.arm.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class FlipRaisedArm extends CommandGroup {

    public FlipRaisedArm(int finalArmState) {
    	if(Robot.armIsFront){
        	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT));

        	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT,Robot.armState,false));
    	}else{
        	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR));

        	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR,Robot.armState,false));
    	}
    	addSequential(new DownLiftUntilSafe());
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
    }
}
