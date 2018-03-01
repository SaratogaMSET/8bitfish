package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.AutoTestCommand;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfile;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.GyroPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 1:
 *****Start on left side and drive forward, turn 90 degrees and drive forward to 
 *****drop off the power cube
 *Position: Left
 */
public class LeftSwitch extends CommandGroup {

    public LeftSwitch() {
    	if(Robot.isTestingAuto) {
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchVal.FIRST_DRIVE));// drive forward
    		addSequential(new GyroPID(AutoTest.LeftSwitchVal.FIRST_ANGLE_TURN));// turn 90 degrees
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchVal.SECOND_DRIVE)); // drive forward
    		addSequential(new GyroPID(AutoTest.LeftSwitchVal.SECOND_ANGLE_TURN));
    		addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchVal.THIRD_DRIVE));
    		addSequential(new AutoTestCommand());
      	// drop off block
    	} else {
    		addSequential(new DrivetrainMotionProfileIn(230.8-Robot.robotLength));// drive forward
    		addSequential(new GyroPID(90));// turn 90 degrees
    		addSequential(new DrivetrainMotionProfileIn(44.6-Robot.robotLength/2)); // drive forward
    		addSequential(new GyroPID(90));
    		addSequential(new DrivetrainMotionProfileIn(21.2-Robot.robotLength/2));
      	// drop off block
    	}
    }
}
