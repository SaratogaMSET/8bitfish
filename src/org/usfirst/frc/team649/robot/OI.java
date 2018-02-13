package org.usfirst.frc.team649.robot;


import org.usfirst.frc.team649.robot.OI.Driver;
import org.usfirst.frc.team649.robot.OI.Operator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

//operater interface check for joystick control
public class OI {
	//individual joysticks
	public Joystick buttonBoard;
	public Joystick driveJoystickHorizontal;
	public Joystick driveJoystickVertical;
	public Joystick operatorJoystick;
	
	public Driver driver;
	public Operator operator;
	
	public boolean oldValue1;
	public boolean oldValue2;
	public boolean oldValue3;
	public boolean oldValue4;
	public boolean oldValue5;
	public boolean oldValue6;
	public boolean oldValue7;
	public boolean oldValue8;
	public boolean oldValue9;
	
	public  OI(){
		buttonBoard = new Joystick(RobotMap.BUTTON_BOARD);
		operatorJoystick = new Joystick(RobotMap.OPERATOR_JOYSTICK);
		driveJoystickHorizontal = new Joystick(RobotMap.DRIVE_JOYSTICK_HORIZONTAL);
		driveJoystickVertical = new Joystick(RobotMap.DRIVE_JOYSTICK_VERTICAL);
//		gamePad = new Joystick(3);
		driver = new Driver();
		operator = new Operator();
		
		oldValue1 = false;
		oldValue2 = false;
		oldValue3 = false;
		oldValue4 = false;
		oldValue5 = false;
		oldValue6 = false;
		oldValue7 = false;
		oldValue8 = false;
		oldValue9 = false;
	}
	public class Operator {
		public boolean PIDTunePhase() {
			boolean value = buttonBoard.getRawButton(1);
			if(value == true && oldValue1 == false){
				oldValue1 = value;
				return true;
			}
			oldValue1 = value;
			return false;
		}
		public boolean getButton2() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(2);
				if(value == true && oldValue2 == false){
					oldValue2 = value;
					return true;
				}
				oldValue2 = value;
				return false;
			}
			return buttonBoard.getRawButton(2);
		}
		public boolean getButton3() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(3);
				if(value == true && oldValue3 == false){
					oldValue3 = value;
					return true;
				}
				oldValue3 = value;
				return false;
			}
			return buttonBoard.getRawButton(3);
		}
		public boolean getButton4() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(4);
				if(value == true && oldValue4 == false){
					oldValue4 = value;
					return true;
				}
				oldValue4 = value;
				return false;
			}
			return buttonBoard.getRawButton(4);
		}
		public boolean getButton5() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(5);
				if(value == true && oldValue5 == false){
					oldValue5 = value;
					return true;
				}
				oldValue5 = value;
				return false;
			}
			return buttonBoard.getRawButton(5);
		}
		public boolean getButton6() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(6);
				if(value == true && oldValue6 == false){
					oldValue6 = value;
					return true;
				}
				oldValue6 = value;
				return false;
			}
			return buttonBoard.getRawButton(6);
		}
		public boolean getButton7() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(7);
				if(value == true && oldValue7 == false){
					oldValue7 = value;
					return true;
				}
				oldValue7 = value;
				return false;
			}
			return buttonBoard.getRawButton(7);
		}
		public boolean getButton8() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(8);
				if(value == true && oldValue8 == false){
					oldValue8 = value;
					return true;
				}
				oldValue8 = value;
				return false;
			}
			return buttonBoard.getRawButton(8);
		}
		public boolean getButton9() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(9);
				if(value == true && oldValue9 == false){
					oldValue9 = value;
					return true;
				}
				oldValue9 = value;
				return false;
			}
			return buttonBoard.getRawButton(9);
		}
//		public boolean getButton10() {
//			return buttonBoard.getRawButton(10);
//		}
//		public boolean getButton11() {
//			return buttonBoard.getRawButton(11);
//		}
//		public boolean getButton12() {
//			return buttonBoard.getRawButton(12);
//		}
		public boolean getButton13() {
			return buttonBoard.getRawButton(13);
		}
		public boolean startNewPID() {
			return buttonBoard.getRawButton(14);
		}
		public boolean switchToCamera1() {
			return buttonBoard.getRawButton(10);
		}

		public boolean switchToCamera2() {
			return buttonBoard.getRawButton(11);
		}

		public boolean switchToCamera3() {
			return buttonBoard.getRawButton(12);
		}
		public double getOperatorY() {
			if(Math.abs(operatorJoystick.getY()) >= 0.05){
				return operatorJoystick.getY();
			}
			return 0.0;
		}
		public boolean getIntakeForward(){
			return operatorJoystick.getRawButton(5);
		}
		public boolean getIntakeReverse(){
			return operatorJoystick.getRawButton(3);
		}
		public double returnSlider() {
			return operatorJoystick.getRawAxis(3);
		}
		public boolean isIntakeOut(){
			return operatorJoystick.getRawButton(9);
		}
		public boolean isIntakeIn(){
			return operatorJoystick.getRawButton(10);
		}
	}
	public class Driver {
		public double getForward() {
			if (driveJoystickVertical.getY() >= 0.05 || driveJoystickVertical.getY() <= -0.05) {
				return -driveJoystickVertical.getY();
			} else {
				return 0.0;
			}
		}
		public double getRotation() {
			if (driveJoystickHorizontal.getX() >= 0.05 || driveJoystickHorizontal.getX() <= -0.05) {
				return -driveJoystickHorizontal.getX();
			} else {
				return 0.0;
			}
		}
		//hold
		public boolean isVBusOveridePush(){
			return driveJoystickHorizontal.getRawButton(2) || driveJoystickVertical.getRawButton(2);
		}
		//toggle robot starts in vel
		public Boolean switchToVbus(){			
			return driveJoystickHorizontal.getRawButton(4) || driveJoystickVertical.getRawButton(4);
		}
		//only used for normal shift
		public Boolean shiftUp() {
			return driveJoystickHorizontal.getRawButton(1) || driveJoystickVertical.getRawButton(1);
		}
		
		//toggle robot starts in autoshift
		public Boolean switchToNormalShift(){
			return driveJoystickHorizontal.getRawButton(5) || driveJoystickVertical.getRawButton(5);
		}
		//hold
		public Boolean forceLowGear(){
			return driveJoystickHorizontal.getRawButton(3) || driveJoystickVertical.getRawButton(3);

		}
		
	}
}
