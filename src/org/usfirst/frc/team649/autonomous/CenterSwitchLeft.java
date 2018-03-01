package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.AutoTestCommand;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 Autonomous 4:
 *Center Autonomous delivers to Switch on right side
 *****Drive forward, turn and drive diagonally, 
 *****then turn to straight again and drive forward a 
 *****little and drop off block
 *Positions: Center
 */
public class CenterSwitchLeft extends CommandGroup {

    public CenterSwitchLeft() {
    	if(Robot.isTestingAuto) {
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterLeftSwitch.FIRST_DRIVE)); // drive straight
    		addSequential(new GyroPID(AutoTest.CenterLeftSwitch.FIRST_ANGLE_TURN)); // turn ~45 degrees
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterLeftSwitch.SECOND_DRIVE)); // drive straight diagonally
    		addSequential(new GyroPID(AutoTest.CenterLeftSwitch.SECOND_DRIVE));// turn back to straight
    		addSequential(new AutoTestCommand());
    		// deploy
    	} else {
    		addSequential(new DrivetrainMotionProfileIn(35.96-Robot.robotLength)); // drive straight
    		addSequential(new GyroPID(-35)); // turn ~45 degrees
    		addSequential(new DrivetrainMotionProfileIn(107.5)); // drive straight diagonally
    		addSequential(new GyroPID(-35));// turn back to straight
    		// deploy
    	}
    }
}
