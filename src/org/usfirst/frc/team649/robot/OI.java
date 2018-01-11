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
	
	public  OI(){
		buttonBoard = new Joystick(RobotMap.BUTTON_BOARD);
		driveJoystickHorizontal = new Joystick(RobotMap.DRIVE_JOYSTICK_HORIZONTAL);
		driveJoystickVertical = new Joystick(RobotMap.DRIVE_JOYSTICK_VERTICAL);
//		gamePad = new Joystick(3);
		driver = new Driver();
		operator = new Operator();
	}
	public class Operator {
	
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
