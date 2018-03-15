package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.SetMotionMagicParameter;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *Autonomous 2:
 ***** Drive straight and turn then drop off block
 *Position: Right
 */
public class RightFarScale extends CommandGroup {

    public RightFarScale() {
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleFarVal.FIRST_DRIVE));// drive forward
      	addSequential(new GyroPID(AutoTest.RightScaleFarVal.FIRST_ANGLE_TURN));// turn 90 degrees
      	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleFarVal.SECOND_DRIVE)); // drive forward
      	addSequential(new GyroPID(AutoTest.RightScaleFarVal.SECOND_ANGLE_TURN));
      	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleFarVal.THIRD_DRIVE));
    }
}
