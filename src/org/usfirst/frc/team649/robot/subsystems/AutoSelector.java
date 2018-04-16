package org.usfirst.frc.team649.robot.subsystems;

import java.util.HashMap;

import org.usfirst.frc.team649.autonomous.*;
import org.usfirst.frc.team649.autonomous.RightScaleSingleMP;
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
	static final int kSwitchLLPin = 0; // analog input pin TODO: change
	static final int kSwitchLRPin = 1; // analog input pin TODO: change
	static final int kSwitchRLPin = 2; // analog input pin TODO: change
	static final int kSwitchRRPin = 3; // analog input pin TODO: change
	static final int kSwitchRobotPos = 5; // analog input pin for robot pos
											// switch TODO: change
	
	static final double[] LLPin1 = {.004, .006}; //pin furthest to left
	static final double[] LLPin2 = {0.9, 1};
	static final double[] LLPin3 = {1.9, 2};
	static final double[] LLPin4 = {2.9, 3};
	static final double[] LLPin5 = {2.9, 3}; //top pin
	static final double[] LLPin6 = {3.9, 4};
	static final double[] LLPin7 = {4.9, 5};
	static final double[] LLPin8 = {0.3, 0.4}; //pin furthest to right
	
	static final double[] LRPin1 = {.004, .006}; //pin furthest to left
	static final double[] LRPin2 = {0.9, 1};
	static final double[] LRPin3 = {1.9, 2};
	static final double[] LRPin4 = {2.9, 3};
	static final double[] LRPin5 = {2.9, 3}; //top pin
	static final double[] LRPin6 = {3.9, 4};
	static final double[] LRPin7 = {4.9, 5};
	static final double[] LRPin8 = {0.3, 0.4}; //pin furthest to right
	
	static final double[] RLPin1 = {.004, .006}; //pin furthest to left
	static final double[] RLPin2 = {0.9, 1};
	static final double[] RLPin3 = {1.9, 2};
	static final double[] RLPin4 = {2.9, 3};
	static final double[] RLPin5 = {2.9, 3}; //top pin
	static final double[] RLPin6 = {3.9, 4};
	static final double[] RLPin7 = {4.9, 5};
	static final double[] RLPin8 = {0.3, 0.4}; //pin furthest to right
	
	static final double[] RRPin1 = {.004, .006}; //pin furthest to left
	static final double[] RRPin2 = {0.9, 1};
	static final double[] RRPin3 = {1.9, 2};
	static final double[] RRPin4 = {2.9, 3};
	static final double[] RRPin5 = {2.9, 3}; //top pin
	static final double[] RRPin6 = {3.9, 4};
	static final double[] RRPin7 = {4.9, 5};
	static final double[] RRPin8 = {0.3, 0.4}; //pin furthest to right
	
	static final double[] RPPin1 = {.004, .006}; //pin furthest to left
	static final double[] RPPin2 = {0.9, 1};
	static final double[] RPPin3 = {1.9, 2};
	static final double[] RPPin4 = {2.9, 3};
	static final double[] RPPin5 = {2.9, 3}; //top pin
	static final double[] RPPin6 = {3.9, 4};
	static final double[] RPPin7 = {4.9, 5};
	static final double[] RPPin8 = {0.3, 0.4}; //pin furthest to right


	// first char = switch, second = scale; => L/R

	static AnalogInput switchLL; // analog switch for LL
	static AnalogInput switchLR; // LR
	static AnalogInput switchRL; // RL
	static AnalogInput switchRR; // RR

	static AnalogInput switchRobotPos;

	// utility vars
	static final HashMap<String, AnalogInput> switchForFieldConfig = new HashMap<String, AnalogInput>() {
		{ // map 'field data' (string) to physical switch
			put("LL", switchLL);
			put("LR", switchLR);
			put("RL", switchRL);
			put("RR", switchRR);
		}
	};

	public AutoSelector() {
		switchLL = new AnalogInput(kSwitchLLPin); // analog switch for LL
		switchLR = new AnalogInput(kSwitchLRPin); // LR
		switchRL = new AnalogInput(kSwitchRLPin); // RL
		switchRR = new AnalogInput(kSwitchRRPin);
		switchRobotPos = new AnalogInput(kSwitchRobotPos);

	}

	// utility mehtods

	private static String getFieldConfiguration() {
		String gameConfig = DriverStation.getInstance().getGameSpecificMessage().substring(0, 2);
		return gameConfig;
	}
	
	public void selectProgram(){
		
		if(getPinRP(getVoltageRobotPos())==1){
			int check = checkProgram();
		}
		else if(getPinRP(getVoltageRobotPos())==3){
			int check = checkProgram();
		}
		else if(getPinRP(getVoltageRobotPos())==2){
			int check = checkProgram();
		}
		
	}
	
	public int checkProgram(){
		String fieldConf = getFieldConfiguration();
		if(fieldConf.equals("LL")){
			return getPinLL(getVoltageLL());
		}
		else if(fieldConf.equals("RR")){
			return getPinLL(getVoltageRR());
		}
		else if(fieldConf.equals("LR")){
			return getPinLL(getVoltageLR());
		}
		else{
			return getPinLL(getVoltageRL());
		}
	}
	
	public int getPinLL(double v){
		if(v>=LLPin1[0] && v<=LLPin1[1]){
			return 1;
		}
		if(v>=LLPin2[0] && v<=LLPin2[1]){
			return 2;
		}
		if(v>=LLPin3[0] && v<=LLPin3[1]){
			return 3;
		}
		if(v>=LLPin4[0] && v<=LLPin4[1]){
			return 4;
		}
		if(v>=LLPin5[0] && v<=LLPin5[1]){
			return 5;
		}
		if(v>=LLPin6[0] && v<=LLPin6[1]){
			return 6;
		}
		if(v>=LLPin7[0] && v<=LLPin7[1]){
			return 7;
		}
		if(v>=LLPin8[0] && v<=LLPin8[1]){
			return 8;
		}
		return -1;
	}
	
	public int getPinLR(double v){
		if(v>=LRPin1[0] && v<=LRPin1[1]){
			return 1;
		}
		if(v>=LRPin2[0] && v<=LRPin2[1]){
			return 2;
		}
		if(v>=LRPin3[0] && v<=LRPin3[1]){
			return 3;
		}
		if(v>=LRPin4[0] && v<=LRPin4[1]){
			return 4;
		}
		if(v>=LRPin5[0] && v<=LRPin5[1]){
			return 5;
		}
		if(v>=LRPin6[0] && v<=LRPin6[1]){
			return 6;
		}
		if(v>=LRPin7[0] && v<=LRPin7[1]){
			return 7;
		}
		if(v>=LRPin8[0] && v<=LRPin8[1]){
			return 8;
		}
		return -1;
	}
	
	public int getPinRL(double v){
		if(v>=RLPin1[0] && v<=RLPin1[1]){
			return 1;
		}
		if(v>=RLPin2[0] && v<=RLPin2[1]){
			return 2;
		}
		if(v>=RLPin3[0] && v<=RLPin3[1]){
			return 3;
		}
		if(v>=RLPin4[0] && v<=RLPin4[1]){
			return 4;
		}
		if(v>=RLPin5[0] && v<=RLPin5[1]){
			return 5;
		}
		if(v>=RLPin6[0] && v<=RLPin6[1]){
			return 6;
		}
		if(v>=RLPin7[0] && v<=RLPin7[1]){
			return 7;
		}
		if(v>=RLPin8[0] && v<=RLPin8[1]){
			return 8;
		}
		return -1;
	}
	
	public int getPinRR(double v){
		if(v>=RRPin1[0] && v<=RRPin1[1]){
			return 1;
		}
		if(v>=RRPin2[0] && v<=RRPin2[1]){
			return 2;
		}
		if(v>=RRPin3[0] && v<=RRPin3[1]){
			return 3;
		}
		if(v>=RRPin4[0] && v<=RRPin4[1]){
			return 4;
		}
		if(v>=RRPin5[0] && v<=RRPin5[1]){
			return 5;
		}
		if(v>=RRPin6[0] && v<=RRPin6[1]){
			return 6;
		}
		if(v>=RRPin7[0] && v<=RRPin7[1]){
			return 7;
		}
		if(v>=RRPin8[0] && v<=RRPin8[1]){
			return 8;
		}
		return -1;
	}
	
	public int getPinRP(double v){
		if(v>=RPPin1[0] && v<=RPPin1[1]){
			return 1;
		}
		if(v>=RPPin2[0] && v<=RPPin2[1]){
			return 2;
		}
		if(v>=RPPin3[0] && v<=RPPin3[1]){
			return 3;
		}
		return -1;
	}

	public double getVoltageLL() {
		return switchLL.getVoltage();
	}

	public double getVoltageLR() {
		return switchLR.getVoltage();
	}

	public double getVoltageRR() {
		return switchRR.getVoltage();
	}

	public double getVoltageRL() {
		return switchRL.getVoltage();
	}

	public double getVoltageRobotPos() {
		return switchRobotPos.getVoltage();
	}

	// MAIN METHOD OF CLASS
	public static Class getAutoClass() {
		String fieldConf = getFieldConfiguration();
		AnalogInput correctSwitch = switchForFieldConfig.get(fieldConf);
		double voltage = correctSwitch.getVoltage();
		double robotPosVoltage = switchRobotPos.getVoltage();

		int robotPosSwitchNum = (int) Math.floor(robotPosVoltage * ((double) VOLTS_PER_SWITCH / TICKS_PER_SWITCH)); // start
																													// zone
																													// number
																													// of
																													// robot
																													// (0-2)
		int autoRunNum = (int) Math.floor(voltage * ((double) VOLTS_PER_SWITCH / TICKS_PER_SWITCH)); // which
																										// auto
																										// to
																										// run
																										// (scale,
																										// double
																										// scale,
																										// switch,
																										// etc.)

		SmartDashboard.putNumber("ROBOT POS SWITCH", robotPosSwitchNum);
		SmartDashboard.putNumber("AUTO RUN NUM", autoRunNum);

		/*
		 * structure: regardless of where you start, a certain number switch
		 * will give you a scoring run. if that run isnt available (switch, the
		 * field config is far) then just drive straight. 0 - drive straight 1 -
		 * single switch if near, drivestraight if far 2 - switch scale, if
		 * switch and scale arent near, drive straight 3 - single scale 4 -
		 * double scale
		 */

		// all if statements are basically " if near"
		switch (robotPosSwitchNum) {
		case 0:
			// robot left
			switch (autoRunNum) {
			// physical switch seting
			case 0:
				return DriveStraight.class;
			case 1:
				if (fieldConf.charAt(0) == 'L')
					return LeftSwitch.class;
				return DriveStraight.class;
			case 2:
				if (fieldConf.equals("LL"))
					return LeftScaleSWSCMP.class;
				return DriveStraight.class;
			case 3:
				if (fieldConf.charAt(1) == 'L') {
					// left near scale single
					return LeftScaleSingleMP.class;
				} else {
					// left far scale single
					return LeftFarScale.class;
				}
			case 4:
				if (fieldConf.charAt(1) == 'L') {
					// left near scale double
					return LeftScaleDoubleScaleMP.class;
				} else {
					// TODO: change to left far scale double
					return DriveStraight.class;
				}
			}
		case 1:
			// robot middle
			switch (autoRunNum) {
			// physical switch seting
			case 0:
				return DriveStraight.class;
			case 1:
				if (fieldConf.charAt(0) == 'L') {
					return LeftMPSwitch.class;
				} else {
					return RightMPSwitch.class;
				}
			default:
				return DriveStraight.class;
			}
		case 2:
			// robot right
			switch (autoRunNum) {
			// physical switch seting
			case 0:
				return DriveStraight.class;
			case 1:
				if (fieldConf.charAt(0) == 'R')
					return RightSwitch.class;

				// cant do far switch, so drive straight
				return DriveStraight.class;
			case 2:
				if (fieldConf.equals("RR"))
					return RighScaleSWSCMP.class;
				return DriveStraight.class;
			case 3:
				if (fieldConf.charAt(1) == 'R') {
					// right near scale single
					return RightScaleSingleMP.class;
				} else {
					// right far scale single
					return RightFarScale.class;
				}
			case 4:
				if (fieldConf.charAt(1) == 'R') {
					// right near scale double
					return RightScaleDoubleScaleMP.class;
				} else {
					// TODO: change to right far scale double
					return DriveStraight.class;
				}
			}
		}

		return DriveStraight.class;

		// TODO: USE voltage (holds voltage for the potentiometer corresponding
		// to the current field config) and robotPosSwitchNum (holds the robot
		// start zone #, 0-indexed)

	}
}
