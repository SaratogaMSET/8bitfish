package org.usfirst.frc.team649.test;

import org.usfirst.frc.team649.robot.commands.Delay;
import org.usfirst.frc.team649.robot.commands.GyroPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class GyroPIDTest extends CommandGroup {

    public GyroPIDTest(double angle, int trials) {
    	for (int i = 0; i < trials; i ++) {
    		addSequential(new GyroPID(angle));
    		addSequential(new Delay(2));
    	}
    }
}
