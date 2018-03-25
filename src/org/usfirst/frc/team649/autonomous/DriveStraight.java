package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous 5:
 * Just drive straight
 * Position: any
 */
public class DriveStraight extends CommandGroup {

    public DriveStraight() {
    	addSequential(new DrivetrainMotionProfileIn(1200)); // Drive Straight
    }
}
