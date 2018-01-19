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
	
	public Driver driver;
	public Operator operator;
	
	public boolean oldValue;
	
	public  OI(){
		buttonBoard = new Joystick(RobotMap.BUTTON_BOARD);
		driveJoystickHorizontal = new Joystick(RobotMap.DRIVE_JOYSTICK_HORIZONTAL);
		driveJoystickVertical = new Joystick(RobotMap.DRIVE_JOYSTICK_VERTICAL);
//		gamePad = new Joystick(3);
		driver = new Driver();
		operator = new Operator();
		
		oldValue = false;
	}
	public class Operator {
		public boolean PIDTunePhase() {
			boolean value = buttonBoard.getRawButton(1);
			if(value == true && oldValue == false){
				oldValue = value;
				return true;
			}
			oldValue = value;
			return false;
		}
		public boolean getButton2() {
			return buttonBoard.getRawButton(2);
		}
		public boolean getButton3() {
			return buttonBoard.getRawButton(3);
		}
		public boolean getButton4() {
			return buttonBoard.getRawButton(4);
		}
		public boolean getButton5() {
			return buttonBoard.getRawButton(5);
		}
		public boolean getButton6() {
			return buttonBoard.getRawButton(6);
		}
		public boolean getButton7() {
			return buttonBoard.getRawButton(7);
		}
		public boolean getButton8() {
			return buttonBoard.getRawButton(8);
		}
		public boolean getButton9() {
			return buttonBoard.getRawButton(9);
		}
		public boolean getButton10() {
			return buttonBoard.getRawButton(10);
		}
		public boolean getButton11() {
			return buttonBoard.getRawButton(11);
		}
		public boolean getButton12() {
			return buttonBoard.getRawButton(12);
		}
		public boolean getButton13() {
			return buttonBoard.getRawButton(13);
		}
		public boolean startNewPID() {
			return buttonBoard.getRawButton(14);
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
