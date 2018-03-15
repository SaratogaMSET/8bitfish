package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetMotionMagicParameter;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *Autonomous 2:
 ***** Drive straight and turn then drop off block
 *Position: Right
 */
public class RightScale extends CommandGroup {

    public RightScale() {
    	addSequential(new SetMotionMagicParameter(AutoTest.LongMotionMagicValues.acceleration,
    			AutoTest.LongMotionMagicValues.velocity, AutoTest.LongMotionMagicValues.k_p,
    			AutoTest.LongMotionMagicValues.k_i, AutoTest.LongMotionMagicValues.k_d));
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleVal.FIRST_DRIVE));// drive forward
      	addSequential(new GyroPID(AutoTest.RightScaleVal.FIRST_ANGLE_TURN)); // turn 90 degrees
//      	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE,Robot.liftState));
      	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleVal.SECOND_DRIVE)); // drive forward
      	addSequential(new GyroPID(AutoTest.RightScaleVal.SECOND_ANGLE_TURN));
      	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleVal.THIRD_DRIVE));
//    	addSequential(new DeployWithWheelsAndIntake()); // deploy
//    	addSequential(new Delay(2));
//    	addSequential(new RunIntakeWheels(0));
    }
}
