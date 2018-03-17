package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.ChangeRobotLiftState;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
import org.usfirst.frc.team649.robot.commands.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetMotionMagicParameter;
import org.usfirst.frc.team649.robot.commands.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
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
    	addSequential(new ZeroArmRoutine());
    	addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleVal.FIRST_DRIVE)); // drive straight
    	addSequential(new GyroPID(AutoTest.RightScaleVal.FIRST_ANGLE_TURN)); // turn ~45 degrees
        addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleVal.SECOND_DRIVE));// drive straight
        addSequential(new ChangeRobotLiftState(LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE));
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT));
        addParallel(new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE,Robot.liftState,1.25));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT,Robot.armState));
        addSequential(new GyroPID(AutoTest.RightScaleVal.SECOND_ANGLE_TURN));
        addSequential(new DrivetrainMotionProfileIn(AutoTest.RightScaleVal.THIRD_DRIVE));
        addSequential(new Delay(0.5));// drive straight
    	addSequential(new RunIntakeForTime(1, false));
    	addSequential(new DrivetrainMotionProfileIn(-10));// deploy
    }
}
