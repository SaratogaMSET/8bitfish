package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.commands.drivetrain.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.drivetrain.GyroPID;
import org.usfirst.frc.team649.robot.commands.intake.RunIntakeForTime;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 3: Left Switch Around
 ***** Drive forward beyond the switch and goes around the back to drop cube
 *Position Left
 */
public class LeftSwitchAround extends CommandGroup {

    public LeftSwitchAround() {
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchFarVal.FIRST_DRIVE)); // drive forward
    	addSequential(new GyroPID(AutoTest.LeftSwitchFarVal.FIRST_ANGLE_TURN)); // turn 90 degrees
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchFarVal.SECOND_DRIVE));// drive forward
    	addSequential(new GyroPID(AutoTest.LeftSwitchFarVal.SECOND_ANGLE_TURN)); // turn 90 degrees
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftSwitchFarVal.THIRD_DRIVE));// drive forward
    	addSequential(new RunIntakeForTime(1, false));
    	// drop cube
        
    }
}
