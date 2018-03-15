package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.test.AutoTestCommand;

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
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterLeftSwitch.FIRST_DRIVE)); // drive straight
    	addSequential(new GyroPID(AutoTest.CenterLeftSwitch.FIRST_ANGLE_TURN)); // turn ~45 degrees
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.CenterLeftSwitch.SECOND_DRIVE)); // drive straight diagonally
    	addSequential(new GyroPID(AutoTest.CenterLeftSwitch.SECOND_ANGLE_TURN));// turn back to straight
    	addSequential(new DeployWithWheelsAndIntake()); // deploy
    	addSequential(new Delay(2));
    	addSequential(new RunIntakeWheels(0));
    	addSequential(new AutoTestCommand());

    }
}
