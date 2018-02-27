package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfile;
import org.usfirst.frc.team649.robot.commands.GyroPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
*Autonomous 1:
 *****Start on right side and drive forward, turn 90 degrees and drive forward to 
 *****drop off the power cube
 *Position: Right
 */
public class RightSwitch extends CommandGroup {

    public RightSwitch() {
    	if(Robot.isTestingAuto) {
    		addSequential(new DrivetrainMotionProfile(AutoTest.RightSwitchVal.FIRST_DRIVE));// drive forward
      		addSequential(new GyroPID(AutoTest.RightSwitchVal.FIRST_ANGLE_TURN));// turn 90 degrees
      		addSequential(new DrivetrainMotionProfile(AutoTest.RightSwitchVal.SECOND_DRIVE)); // drive forward
      		addSequential(new GyroPID(AutoTest.RightSwitchVal.SECOND_ANGLE_TURN));
      		addSequential(new DrivetrainMotionProfile(AutoTest.RightSwitchVal.THIRD_DRIVE));
      		// drop off block
    	} else {
    		addSequential(new DrivetrainMotionProfile(230.8-Robot.robotLength));// drive forward
      		addSequential(new GyroPID(-90));// turn 90 degrees
      		addSequential(new DrivetrainMotionProfile(44.6-Robot.robotLength/2)); // drive forward
      		addSequential(new GyroPID(-90));
      		addSequential(new DrivetrainMotionProfile(21.2-Robot.robotLength/2));
      		// drop off block
    	}
    }
}
