package org.usfirst.frc.team649.robot.subsystems;

import java.util.HashMap;

import org.usfirst.frc.team649.autonomous.*;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.CommandGroups.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class AutoSelector {
		static final int TICKS_PER_SWITCH = 8;
		static final int VOLTS_PER_SWITCH = 5;

	// TODO: UPDATE WITH REAL PIN VALUES
		static final int kSwitchLLPin = 1; // analog input pin TODO: change
		static final int kSwitchLRPin = 2; // analog input pin TODO: change
		static final int kSwitchRLPin = 3; // analog input pin TODO: change
		static final int kSwitchRRPin = 4; // analog input pin TODO: change
		static final int kSwitchRobotPos = 5; //analog input pin for robot pos switch TODO: change
		
		// first char = switch, second = scale; => L/R

		static final AnalogInput switchLL = new AnalogInput(kSwitchLLPin); // analog switch for LL
		static final AnalogInput switchLR = new AnalogInput(kSwitchLRPin); // LR 
		static final AnalogInput switchRL = new AnalogInput(kSwitchRLPin); // RL
		static final AnalogInput switchRR = new AnalogInput(kSwitchRRPin); // RR
		
		static final AnalogInput switchRobotPos = new AnalogInput(kSwitchRobotPos);
		
		// utility vars	
		static final HashMap<String, AnalogInput> switchForFieldConfig = new HashMap<String, AnalogInput>() {{ // map 'field data' (string) to physical switch
			put("LL", switchLL);
			put("LR", switchLR);
			put("RL", switchRL);
			put("RR", switchRR);
		}};
		
		
		// utility mehtods
		
		private static String getFieldConfiguration() {
			String gameConfig = DriverStation.getInstance().getGameSpecificMessage().substring(0, 2);
			return gameConfig;
		}
		
		
		
		// MAIN METHOD OF CLASS
		public static Class getAutoClass() {
			String fieldConf = getFieldConfiguration();
			AnalogInput correctSwitch = switchForFieldConfig.get(fieldConf);
			double voltage = correctSwitch.getVoltage();
			double robotPosVoltage = switchRobotPos.getVoltage();
			
			
			int robotPosSwitchNum = (int) Math.floor(robotPosVoltage * ((double)VOLTS_PER_SWITCH/TICKS_PER_SWITCH)); // start zone number of robot (0-2)
			int autoRunNum = (int) Math.floor(voltage * ((double)VOLTS_PER_SWITCH/TICKS_PER_SWITCH)); // which auto to run (scale, double scale, switch, etc.)

			SmartDashboard.putNumber("ROBOT POS SWITCH", robotPosSwitchNum);
			SmartDashboard.putNumber("AUTO RUN NUM", autoRunNum);

			/*
			 * structure: regardless of where you start, a certain number switch will give you a scoring run. if that run isnt available (switch, the field config is far) then just drive straight.
			 * 0 - drive straight
			 * 1 - single switch if near, drivestraight if far
			 * 2 - switch scale, if switch and scale arent near, drive straight
			 * 3 - single scale
			 * 4 - double scale
			 */
			
			// all if statements are basically " if near"
			switch(robotPosSwitchNum) {
			case 0:
				// robot left
				switch(autoRunNum) {
				// physical switch seting
				case 0:
					return DriveStraight.class;
				case 1:
					if(fieldConf.charAt(0) == 'L')
						return LeftSwitch.class;
					return DriveStraight.class;
				case 2:
					if(fieldConf.equals("LL"))
						return LeftScaleSWSCMP.class;
					return DriveStraight.class;
				case 3:
					if(fieldConf.charAt(1) == 'L') {
						// left near scale single
						return LeftScaleSingleMP.class;
					}
					else {
						// left far scale single
						return LeftFarScale.class;
					}
				case 4:
					if(fieldConf.charAt(1) == 'L') {
						// left near scale double
						return LeftScaleDoubleScaleMP.class;
					}
					else {
						// TODO: change to left far scale double
						return DriveStraight.class;
					}
				}
			case 1:
				// robot middle
				switch(autoRunNum) {
				// physical switch seting
				case 0:
					return DriveStraight.class;
				case 1:
					if(fieldConf.charAt(0) == 'L') {
						return LeftMPSwitch.class;
					}
					else {
						return RightMPSwitch.class;
					}
				default:
					return DriveStraight.class;
				}
			case 2:
				// robot right
				switch(autoRunNum) {
				// physical switch seting
				case 0:
					return DriveStraight.class;
				case 1:
					if(fieldConf.charAt(0) == 'R')
						return RightSwitch.class;
					
					// cant do far switch, so drive straight
					return DriveStraight.class;
				case 2:
					if(fieldConf.equals("RR"))
						return RighScaleSWSCMP.class;
					return DriveStraight.class;
				case 3:
					if(fieldConf.charAt(1) == 'R') {
						// right near scale single
						return RightScaleSingleMP.class;
					}
					else {
						// right far scale single
						return RightFarScale.class;
					}
				case 4:
					if(fieldConf.charAt(1) == 'R') {
						// right near scale double
						return RightScaleDoubleScaleMP.class;
					}
					else {
						// TODO: change to right far scale double
						return DriveStraight.class;
					}
				}
			}
			
			return DriveStraight.class;
			
			// TODO: USE voltage (holds voltage for the potentiometer corresponding to the current field config) and robotPosSwitchNum (holds the robot start zone #, 0-indexed)
			
		}
}

