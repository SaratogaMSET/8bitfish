package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 2:
 ***** Drive straight and turn then drop off block
 *Position: Left
 */
public class LeftScale extends CommandGroup {

    public LeftScale() {
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleVal.FIRST_DRIVE)); // drive straight
    	addSequential(new GyroPID(AutoTest.LeftScaleVal.FIRST_ANGLE_TURN)); // turn ~45 degrees
    	int liftState = LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE;
      	addSequential(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE,liftState));
        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleVal.SECOND_DRIVE));// drive straight
    	addSequential(new GyroPID(AutoTest.LeftScaleVal.SECOND_ANGLE_TURN));
        addSequential(new DrivetrainMotionProfileIn(AutoTest.LeftScaleVal.THIRD_DRIVE));// drive straight
//    	addSequential(new DeployWithWheelsAndIntake()); // deploy
//    	addSequential(new Delay(2));
//    	addSequential(new RunIntakeWheels(0));
        addSequential(new AutoTestCommand());
    	// deploy
    }
}
