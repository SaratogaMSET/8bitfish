package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class AutoSelector extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public AnalogPotentiometer[] pots;

	public AutoSelector(){
		pots[0] = new AnalogPotentiometer(RobotMap.AutoSwitches.Auto_Switches[0]);
		pots[1] = new AnalogPotentiometer(RobotMap.AutoSwitches.Auto_Switches[1]);
		pots[2] = new AnalogPotentiometer(RobotMap.AutoSwitches.Auto_Switches[2]);
		pots[3] = new AnalogPotentiometer(RobotMap.AutoSwitches.Auto_Switches[3]);
		pots[4] = new AnalogPotentiometer(RobotMap.AutoSwitches.Auto_Switches[4]);
	}
	public double getPot1(){
		return pots[0].get();
	}
	public double getPot2(){
		return pots[1].get();
	}
	public double getPot3(){
		return pots[2].get();
	}
	public double getPot4(){
		return pots[3].get();
	}
	public double getPotSide(){
		return pots[4].get();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

