package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.RunIntakeForTime;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 3: Right Switch Around
 ***** Drive forward beyond the switch and goes around the back to drop cube
 *Position: Right
 */
public class RightSwitchAround extends CommandGroup {

    public RightSwitchAround() {
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightSwitchFarVal.FIRST_DRIVE)); // drive forward
    	addSequential(new GyroPID(AutoTest.RightSwitchFarVal.FIRST_ANGLE_TURN)); // turn 90 degrees
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightSwitchFarVal.SECOND_DRIVE));// drive forward
    	addSequential(new GyroPID(AutoTest.RightSwitchFarVal.SECOND_ANGLE_TURN)); // turn 90 degrees
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightSwitchFarVal.THIRD_DRIVE));// drive forward
    	addSequential(new RunIntakeForTime(1, false));

    	// drop cube
        
    }
}
