package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 2:
 ***** Drive straight and turn then drop off block
 *Position: Right
 */
public class RightScale extends CommandGroup {

    public RightScale() {
        addSequential(new DrivetrainPIDCommand(30)); // drive straight
    	addSequential(new GyroPID(45)); // turn ~45 degrees
        addSequential(new DrivetrainPIDCommand(30));// drive straight
    	// deploy
    }
}
