package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 2:
 ***** Drive straight and turn then drop off block
 *Position: Left
 */
public class LeftFarScale extends CommandGroup {

    public LeftFarScale() {
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.FIRST_DRIVE)); // drive straight
    	addSequential(new GyroPID(AutoTest.LeftScaleFarVal.FIRST_ANGLE_TURN)); // turn ~45 degrees
        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.SECOND_DRIVE));// drive straight
    	addSequential(new GyroPID(AutoTest.LeftScaleFarVal.SECOND_ANGLE_TURN));
        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleFarVal.THIRD_DRIVE));// drive straight
    	addSequential(new AutoTestCommand());
    	// deploy
    }
}
