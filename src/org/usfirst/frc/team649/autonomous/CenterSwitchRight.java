package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.AutoTestCommand;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
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
    	if(Robot.isTestingAuto) {
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterRightSwitch.FIRST_DRIVE)); // drive straight
    		addSequential(new GyroPID(AutoTest.CenterRightSwitch.FIRST_ANGLE_TURN)); // turn ~45 degrees
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterRightSwitch.SECOND_DRIVE)); // drive straight diagonally
    		addSequential(new GyroPID(AutoTest.CenterRightSwitch.SECOND_ANGLE_TURN));// turn back to straight
    		addSequential(new AutoTestCommand());
    		// deploy
    	} else {
    		addSequential(new DrivetrainMotionProfileIn(35.96-Robot.robotLength)); // drive straight
    		addSequential(new GyroPID(-30)); // turn ~45 degrees
    		addSequential(new DrivetrainMotionProfileIn(95.85)); // drive straight diagonally
    		addSequential(new GyroPID(30));// turn back to straight
    		// deploy
    	}
    }
}
