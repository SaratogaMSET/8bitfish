package org.usfirst.frc.team649.robot.CommandGroups;

import org.usfirst.frc.team649.robot.commands.intake.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.intake.SetIntakePistons;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DeployWithWheelsAndIntake extends CommandGroup {

    public DeployWithWheelsAndIntake() {
    	addParallel(new RunIntakeWheels(-1));
    	addParallel(new SetIntakePistons(true,false));
    }
}
