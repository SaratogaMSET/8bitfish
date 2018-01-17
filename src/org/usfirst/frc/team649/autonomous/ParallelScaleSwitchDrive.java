package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Drive Straight for going from the scale to the switch if both on the same side
 *and we have dropped a block in the scale
 *
 *drives backwards a distance and picks up and drops off a block
 */
public class ParallelScaleSwitchDrive extends CommandGroup {

    public ParallelScaleSwitchDrive() {
        addSequential(new DrivetrainPIDCommand(-49)); // drive backwards
        // move arms backwards
        // pick up block
        // deploy
    }
}
