package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfile;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
*Autonomous 1:
 *****Start on right side and drive forward, turn 90 degrees and drive forward to 
 *****drop off the power cube
 *Position: Right
 */
public class RightSwitch extends CommandGroup {

    public RightSwitch() {
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightSwitchVal.FIRST_DRIVE));// drive forward
      	addSequential(new GyroPID(AutoTest.RightSwitchVal.FIRST_ANGLE_TURN));// turn 90 degrees
      	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightSwitchVal.SECOND_DRIVE)); // drive forward
      	addSequential(new GyroPID(AutoTest.RightSwitchVal.SECOND_ANGLE_TURN));
      	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightSwitchVal.THIRD_DRIVE));
    	addSequential(new DeployWithWheelsAndIntake()); // deploy
    	addSequential(new Delay(2));
    	addSequential(new RunIntakeWheels(0));
      	addSequential(new AutoTestCommand());
      	// drop off block
    }
}
