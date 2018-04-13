package org.usfirst.frc.team649.robot;

import org.usfirst.frc.team649.robot.OI.Driver;
import org.usfirst.frc.team649.robot.OI.Operator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;

//operater interface check for joystick control
public class OI {
	// individual joysticks
	public Joystick buttonBoard;
	public Joystick driveJoystickHorizontal;
	public Joystick driveJoystickVertical;
	public Joystick operatorJoystick;

	public Driver driver;
	public Operator operator;
	public Joystick buttonBoard2;

	public boolean oldValue1;
	public boolean oldValue2;
	public boolean oldValue3;
	public boolean oldValue4;
	public boolean oldValue5;
	public boolean oldValue6;
	public boolean oldValue7;
	public boolean oldValue8;
	public boolean oldValue9;
	public boolean intakeBBPrev;
	public boolean shiftPrev;
	public boolean flipPrev;
	public Timer time;

	public OI() {
		buttonBoard = new Joystick(RobotMap.BUTTON_BOARD);
		time = new Timer();
		buttonBoard2 = new Joystick(RobotMap.BUTTON_BOARD_2);
		operatorJoystick = new Joystick(RobotMap.OPERATOR_JOYSTICK);
		driveJoystickHorizontal = new Joystick(RobotMap.DRIVE_JOYSTICK_HORIZONTAL);
		driveJoystickVertical = new Joystick(RobotMap.DRIVE_JOYSTICK_VERTICAL);
		// gamePad = new Joystick(3);
		driver = new Driver();
		operator = new Operator();
		shiftPrev = false;
		intakeBBPrev = false;
		flipPrev = false;
		
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
			if (value == true && oldValue1 == false) {
				oldValue1 = value;
				return true;
			}
			oldValue1 = value;
			return false;
		}
		public boolean newBBIntake(){
			boolean val = buttonBoard2.getRawButton(4);
			if(val == true && intakeBBPrev == false){
				intakeBBPrev = val;
				return true;
			}
			intakeBBPrev = val;
			return false;
		}
		public void intakePrev(){
			if(buttonBoard2.getRawButton(4) != intakeBBPrev){
				if(time.get() > 0.05){
					intakeBBPrev = buttonBoard2.getRawButton(4);
					time.stop();
					time.reset();
					
				}else{
					time.start();
				}
			}
		}
		public boolean newShiftToggle(){
			boolean val = driver.shiftUp();
			if(val && !shiftPrev){
				shiftPrev = val;
				return true;
			}
			shiftPrev = val;
			return false;
		}
		public boolean lowSpeedDeploy(){
			return buttonBoard2.getRawButton(2);
		}
		// use prev states
		public boolean getLiftUpSmall() {
			return buttonBoard.getRawButton(12);
		}

		public boolean getLiftDownSmall() {
			return buttonBoard.getRawButton(9);
		}

		public boolean getArmUpSmall() {
			return buttonBoard.getRawButton(10);
		}

		public boolean getArmDownSmall() {
			return buttonBoard.getRawButton(11);
		}

		public boolean getIntakeState() {
			return buttonBoard.getRawButton(8);
		}

		public boolean getStoreState() {
			return buttonBoard.getRawButton(5);
		}

		public boolean getExchangeState() {
			return buttonBoard.getRawButton(7);
		}

		public boolean getSwitchState() {
			return buttonBoard.getRawButton(4);
		}

		public boolean getScaleLowState() {
			return buttonBoard.getRawButton(4);
		}

		public boolean getScaleMidState() {
			return buttonBoard.getRawButton(3);
		}

		public boolean getScaleHighState() {
			return buttonBoard.getRawButton(2);
		}

		public boolean deployWithWheelsAndOpen() {
			return operatorJoystick.getRawButton(1);
		}

		public boolean closeIntake() {
			return operatorJoystick.getRawButton(4);
		}

		public boolean runIntakeWithWheelsClosed() {
			return operatorJoystick.getRawButton(2) || buttonBoard2.getRawButton(3);
		}

		public boolean openIntakeToggle() {
			return buttonBoard2.getRawButton(4);
		}
		
		public boolean openIntakeToggleBB(){
			 return buttonBoard2.getRawButton(4);
		}

		public boolean deployOnlyWheels() {
			return operatorJoystick.getRawButton(5)||buttonBoard2.getRawButton(1);
		}

		public boolean flipArm() {
			return operatorJoystick.getRawButton(6);
		}
		public boolean flipAndStore(){
			return buttonBoard2.getRawButton(5);
		}
		public boolean flipAndIntakeHigh(){
			return buttonBoard2.getRawButton(7);
		}
		public boolean flipAndIntakeLow(){
			boolean value = buttonBoard2.getRawButton(8);
			if (value == true && flipPrev == false) {
				flipPrev = value;
				return true;
			}
			flipPrev = value;
			return false;
		}
		public boolean isManual() {
			return operatorJoystick.getRawButton(12);
		}

		public double getManualArm() {
			return operatorJoystick.getRawAxis(0);
		}

		public double getManualLift() {
			return operatorJoystick.getRawAxis(1);
		}

		public boolean getButton2() {
			if (Robot.isTuningPID) {
				boolean value = buttonBoard.getRawButton(2);
				if (value == true && oldValue2 == false) {
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
				if (value == true && oldValue3 == false) {
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
				if (value == true && oldValue4 == false) {
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
				if (value == true && oldValue5 == false) {
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
				if (value == true && oldValue6 == false) {
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
				if (value == true && oldValue7 == false) {
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
				if (value == true && oldValue8 == false) {
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
				if (value == true && oldValue9 == false) {
					oldValue9 = value;
					return true;
				}
				oldValue9 = value;
				return false;
			}
			return buttonBoard.getRawButton(9);
		}

		// public boolean getButton10() {
		// return buttonBoard.getRawButton(10);
		// }
		// public boolean getButton11() {
		// return buttonBoard.getRawButton(11);
		// }
		// public boolean getButton12() {
		// return buttonBoard.getRawButton(12);
		// }
		public boolean getButton13() {
			return buttonBoard.getRawButton(13);
		}

		public boolean startNewPID() {
			return buttonBoard.getRawButton(14);
		}

		public boolean switchToCamera1() {
			if (RobotMap.Camera.practiceBot) {
				return operatorJoystick.getRawButton(10);
			} else {
				return false;
			}
		}

		// public boolean switchToCamera3() {
		// if(RobotMap.Camera.practiceBot) {
		// return operatorJoystick.getRawButton(12);
		// }
		// else {
		// return false;
		// }
		// }
		public boolean switchToCamera4() {
			if (RobotMap.Camera.practiceBot == false) {
				return operatorJoystick.getRawButton(6);

			} else {
				return false;
			}
		}

		public boolean switchToCamera2() {
			return buttonBoard2.getRawButton(11);
		}

		public boolean switchToCamera5() {
			if (RobotMap.Camera.practiceBot == false) {
				return buttonBoard2.getRawButton(11);

			} else {
				return false;
			}
		}

		// public boolean switchToCamera6() {
		// if(RobotMap.Camera.practiceBot == false)
		// {
		// return buttonBoard.getRawButton(12);
		//
		// }
		// else {
		// return false;
		// }
		// }
		public double getOperatorY() {
			if (Math.abs(operatorJoystick.getY()) >= 0.1) {
				return operatorJoystick.getY();
			}
			return 0.0;
		}

		public boolean getIntakeForward() {
			return operatorJoystick.getRawButton(5);
		}

		public boolean getIntakeReverse() {
			return operatorJoystick.getRawButton(3);
		}

		public double returnSlider() {
			return operatorJoystick.getRawAxis(3);
		}

		public boolean isIntakeOut() {
			return operatorJoystick.getRawButton(7);
		}

		public boolean isIntakeIn() {
			return operatorJoystick.getRawButton(8);
		}

		public boolean getButton2Operator() {
			boolean value = operatorJoystick.getRawButton(2);
			if (value == true && oldValue2 == false) {
				oldValue2 = value;
				return true;
			}
			oldValue2 = value;
			return false;
		}
	}

	public class Driver {
		public double getForward() {
			if (driveJoystickVertical.getY() >= 0.05 || driveJoystickVertical.getY() <= -0.05) {
				return -driveJoystickVertical.getY();
			} else {
				return 0;
			}
		}

		public boolean switchToCamera2() {
			if (RobotMap.Camera.practiceBot) {
				return operatorJoystick.getRawButton(11);
			} else {
				return false;
			}

		}

		// public boolean switchToCamera3() {
		// if(RobotMap.Camera.practiceBot) {
		// return operatorJoystick.getRawButton(12);
		// }
		// else {
		// return false;
		// }
		// }
		public boolean switchToCamera4() {
			if (RobotMap.Camera.practiceBot == false) {
				return buttonBoard.getRawButton(10);

			} else {
				return false;
			}
		}

		public boolean switchToCamera5() {
			if (RobotMap.Camera.practiceBot == false) {
				return buttonBoard.getRawButton(11);

			} else {
				return false;
			}
		}

		// public boolean switchToCamera6() {
		// if(RobotMap.Camera.practiceBot == false)
		// {
		// return buttonBoard.getRawButton(12);
		//
		// }
		// else {
		// return false;
		// }
		// }
		public double getOperatorY() {
			if (Math.abs(operatorJoystick.getY()) >= 0.1) {
				return operatorJoystick.getY();
			}
			return 0.0;
		}

		public double getRotation() {
			if (driveJoystickHorizontal.getX() >= 0.05 || driveJoystickHorizontal.getX() <= -0.05) {
				return -driveJoystickHorizontal.getX();
			} else {
				return 0.0;
			}
		}

		// hold
		public boolean isVBusOveridePush() {
			return driveJoystickHorizontal.getRawButton(2) || driveJoystickVertical.getRawButton(2);
		}

		// toggle robot starts in vel
		public Boolean switchToVbus() {
			return driveJoystickHorizontal.getRawButton(4) || driveJoystickVertical.getRawButton(4);
		}

		// only used for normal shift
		public Boolean shiftUp() {
			return driveJoystickHorizontal.getRawButton(1) || driveJoystickVertical.getRawButton(1);
		}

		// toggle robot starts in autoshift
		public Boolean switchToNormalShift() {
			return driveJoystickHorizontal.getRawButton(5) || driveJoystickVertical.getRawButton(5);
		}

		// hold
		public Boolean forceLowGear() {
			return driveJoystickHorizontal.getRawButton(3) || driveJoystickVertical.getRawButton(3);

		}
	}
}
