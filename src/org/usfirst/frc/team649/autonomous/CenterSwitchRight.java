package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 4:
 *Center Autonomous delivers to Switch on right side
 *****Drive forward, turn and drive diagonally, 
 *****then turn to straight again and drive forward a 
 *****little and drop off block
 *Positions: Center
 */
public class CenterSwitchRight extends CommandGroup {

    public CenterSwitchRight() {
    	addSequential(new DrivetrainPIDCommand(30)); // drive straight
    	addSequential(new GyroPID(45)); // turn ~45 degrees
    	addSequential(new DrivetrainPIDCommand(30)); // drive straight diagonally
    	addSequential(new GyroPID(45));// turn back to straight
    	addSequential(new DrivetrainPIDCommand(30)); // drive forward
    	// deploy
    }
}
