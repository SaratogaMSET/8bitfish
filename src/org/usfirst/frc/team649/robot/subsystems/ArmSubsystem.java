package org.usfirst.frc.team649.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ArmSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	DigitalInput infraredSensor;
	
	public ArmSubsystem() {
		infraredSensor = new DigitalInput(0);
	}
	
	public boolean getInfraredSensor() {
		return infraredSensor.get();
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

