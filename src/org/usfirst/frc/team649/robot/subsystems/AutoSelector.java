package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class AutoSelector extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public AnalogInput[] pots;

	public AutoSelector(){
		pots[0] = new AnalogInput(RobotMap.AutoSwitches.Auto_Switches[0]);
		pots[1] = new AnalogInput(RobotMap.AutoSwitches.Auto_Switches[1]);
		pots[2] = new AnalogInput(RobotMap.AutoSwitches.Auto_Switches[2]);
		pots[3] = new AnalogInput(RobotMap.AutoSwitches.Auto_Switches[3]);
		pots[4] = new AnalogInput(RobotMap.AutoSwitches.Auto_Switches[4]);
	}
	public double getPot1(){
		return pots[0].getVoltage();
	}
	public double getPot2(){
		return pots[1].getVoltage();
	}
	public double getPot3(){
		return pots[2].getVoltage();
	}
	public double getPot4(){
		return pots[3].getVoltage();
	}
	public double getPotSide(){
		return pots[4].getVoltage();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

