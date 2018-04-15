package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
import org.usfirst.frc.team649.robot.commands.ChangeRobotArmState;
import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.DrivetrainMotionProfileIn;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.GyroPID;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive;
import org.usfirst.frc.team649.robot.commands.RunIntakeForTime;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetMotionMagicParameter;
import org.usfirst.frc.team649.robot.commands.ZeroArmRoutine;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team649.test.AutoTestCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *Autonomous 4:
 *Center Autonomous delivers to Switch on right side
 *****Drive forward, turn and drive diagonally, 
 *****then turn to straight again and drive forward a 
 *****little and drop off block
 *Positions: Center
 */
public class CenterSwitchRight extends CommandGroup {

    public CenterSwitchRight() {
    	addSequential(new ChangeRobotArmState(ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT));
    	addParallel(new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT,Robot.armState,false));
        addSequential(new MotionProfileDrive(false));
        addSequential(new RunIntakeForTime(1,false));

    }
}
