package org.usfirst.frc.team649.autonomous;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoTest {
	public static class LongMotionMagicValues {
		public static final int acceleration = 9000;
		public static final int velocity = 18000;
		public static final double k_p = 1.5;
		public static final double k_i = 0;
		public static final double k_d = 0.9;
	}
	public static class AutoPrograms {
		public static final int CENTER_RIGHT_SWITCH = 1;
		public static final int CENTER_LEFT_SWITCH = 2;
		public static final int LEFT_SWITCH = 3;
		public static final int RIGHT_SWITCH = 4;
		public static final int LEFT_SCALE = 5;
		public static final int RIGHT_SCALE = 6;
	}
	
	public static class AutoSegments {
		public static final int FIRST_SEGMENT = 1;
		public static final int SECOND_SEGMENT = 2;
		public static final int THIRD_SEGMENT = 3;
		public static final int FOURTH_SEGMENT = 4;
		public static final int FIFTH_SEGMENT = 5;
	}
	
	public static class CenterRightSwitch {
		public static double FIRST_DRIVE = 35.96-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = 32;
		public static double SECOND_DRIVE = 95.85;
		public static double SECOND_ANGLE_TURN = -32;
	}
	
	public static class CenterLeftSwitch {
		public static double FIRST_DRIVE = 35.96-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = -35;
		public static double SECOND_DRIVE = 102;
		public static double SECOND_ANGLE_TURN = 35;
	}
	
	public static class RightSwitchVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = -90;
		public static double SECOND_DRIVE = 44.6-Robot.robotLength/2;
		public static double SECOND_ANGLE_TURN = -90;
		public static double THIRD_DRIVE = 21.2-Robot.robotLength/2;
	}
	
	public static class RightSwitchFarVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = -90;
		public static double SECOND_DRIVE = 155 + 44.6-Robot.robotLength/2;
		public static double SECOND_ANGLE_TURN = -90;
		public static double THIRD_DRIVE = 21.2-Robot.robotLength/2;
	}
	
	public static class LeftSwitchVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = 90;
		public static double SECOND_DRIVE = 44.6-Robot.robotLength/2;
		public static double SECOND_ANGLE_TURN = 90;
		public static double THIRD_DRIVE = 21.2-Robot.robotLength/2;
	}
	
	public static class LeftSwitchFarVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = 90;
		public static double SECOND_DRIVE = 155 + 44.6-Robot.robotLength/2;
		public static double SECOND_ANGLE_TURN = 90;
		public static double THIRD_DRIVE = 21.2-Robot.robotLength/2;
	}
	
	public static class RightScaleVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = -83; // need to tune
		public static double SECOND_DRIVE = 52.6-Robot.robotLength/2;
		public static double SECOND_ANGLE_TURN = 90;
		public static double THIRD_DRIVE = 69.2-Robot.robotLength/2;
//		public static double FIRST_DRIVE = 250-Robot.robotLength/2;
//		public static double FIRST_ANGLE_TURN = -45;
//		public static double SECOND_DRIVE = 42.41-Robot.robotLength/2;
//		public static double SECOND_ANGLE_TURN = 45;
	}
	
	public static class RightScaleFarVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = -83; // need to tune
		public static double SECOND_DRIVE = 155+52.6-Robot.robotLength/2;
		public static double SECOND_ANGLE_TURN = 90;
		public static double THIRD_DRIVE = 69.2-Robot.robotLength/2;
//		public static double FIRST_DRIVE = 250-Robot.robotLength/2;
//		public static double FIRST_ANGLE_TURN = -45;
//		public static double SECOND_DRIVE = 42.41-Robot.robotLength/2;
//		public static double SECOND_ANGLE_TURN = 45;
	}
	
	public static class LeftScaleVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = 85; // need to tune, should be 90
		public static double SECOND_DRIVE = 52.6-Robot.robotLength/2;// + 155
		public static double SECOND_ANGLE_TURN = -90;
		public static double THIRD_DRIVE = 69.2-Robot.robotLength/2;
	}
	
	public static class LeftScaleFarVal {
		public static double FIRST_DRIVE = 230.8-Robot.robotLength/2;
		public static double FIRST_ANGLE_TURN = 85; // need to tune, should be 90
		public static double SECOND_DRIVE = 155+52.6-Robot.robotLength/2;// + 155
		public static double SECOND_ANGLE_TURN = -90;
		public static double THIRD_DRIVE = 69.2-Robot.robotLength/2;
	}
	
	public int currentlyTuningProgram;
	public int currentlyTuningSegment;
	public double tuningIncrement;
	public double tuningVarValue;
	
	public AutoTest() {
		currentlyTuningProgram = 1;
		tuningVarValue = 0;
		tuningIncrement = 1;
		currentlyTuningSegment = 1;
	}
	
	public void changeTuningIncrement(boolean up) {
		if (up) {
			tuningIncrement *= 10;
		} else {
			tuningIncrement /= 10;
		}
	}
	
	public void changeCurrentlyTuningProgram(boolean next) {
		if (next) {
			if(currentlyTuningProgram != 6) {
				currentlyTuningProgram++;
			} else {
				currentlyTuningProgram = 1;
			}
		} else {
			if(currentlyTuningProgram == 1) {
				currentlyTuningProgram = 6;
			}
			currentlyTuningProgram--;
		}
	}
	public void incrementTuningVarValue(boolean up) {
		if (up) {
			tuningVarValue += tuningIncrement;
		} else {
			tuningVarValue -= tuningIncrement;
		}
	}
	public void changeCurrentlyTuningSegment(boolean next) {
		if (next) {
			if(currentlyTuningSegment != 5) {
				currentlyTuningSegment++;
			} else {
				currentlyTuningSegment = 1;
			}
			selectSegmentForTuning();
		} else {
			if(currentlyTuningSegment == 1) {
				currentlyTuningSegment = 5;
			}
			currentlyTuningSegment--;
			selectSegmentForTuning();
		}
	}
	public void selectSegmentForTuning() {
		switch (currentlyTuningProgram) {
		case AutoPrograms.CENTER_LEFT_SWITCH:
			selectSegmentForTuningCenterLeft();
			break;
		case AutoPrograms.CENTER_RIGHT_SWITCH:
			selectSegmentForTuningCenterRight();
			break;
		case AutoPrograms.LEFT_SCALE:
			selectSegmentForTuningLeftScale();
			break;
		case AutoPrograms.RIGHT_SCALE:
			selectSegmentForTuningRightScale();
			break;
		case AutoPrograms.LEFT_SWITCH:
			selectSegmentForTuningLeftSwitch();
			break;
		case AutoPrograms.RIGHT_SWITCH:
			selectSegmentForTuningRightSwitch();
			break;
	}
	}
	public void selectSegmentForTuningCenterRight() {
		switch(currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				tuningVarValue = CenterRightSwitch.FIRST_DRIVE;
				break;
			case AutoSegments.SECOND_SEGMENT:
				tuningVarValue = CenterRightSwitch.FIRST_ANGLE_TURN;
				break;
			case AutoSegments.THIRD_SEGMENT:
				tuningVarValue = CenterRightSwitch.SECOND_DRIVE;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				tuningVarValue = CenterRightSwitch.SECOND_ANGLE_TURN;
				break;
		}
	}
	
	public void selectSegmentForTuningCenterLeft() {
		switch(currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				tuningVarValue = CenterLeftSwitch.FIRST_DRIVE;
				break;
			case AutoSegments.SECOND_SEGMENT:
				tuningVarValue = CenterLeftSwitch.FIRST_ANGLE_TURN;
				break;
			case AutoSegments.THIRD_SEGMENT:
				tuningVarValue = CenterLeftSwitch.SECOND_DRIVE;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				tuningVarValue = CenterLeftSwitch.SECOND_ANGLE_TURN;
				break;
		}
	}
	
	public void selectSegmentForTuningRightScale() {
		switch(currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				tuningVarValue = RightScaleVal.FIRST_DRIVE;
				break;
			case AutoSegments.SECOND_SEGMENT:
				tuningVarValue = RightScaleVal.FIRST_ANGLE_TURN;
				break;
			case AutoSegments.THIRD_SEGMENT:
				tuningVarValue = RightScaleVal.SECOND_DRIVE;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				tuningVarValue = RightScaleVal.SECOND_ANGLE_TURN;
				break;
		}
	}
	
	public void selectSegmentForTuningLeftScale() {
		switch(currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				tuningVarValue = LeftScaleVal.FIRST_DRIVE;
				break;
			case AutoSegments.SECOND_SEGMENT:
				tuningVarValue = LeftScaleVal.FIRST_ANGLE_TURN;
				break;
			case AutoSegments.THIRD_SEGMENT:
				tuningVarValue = LeftScaleVal.SECOND_DRIVE;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				tuningVarValue = LeftScaleVal.SECOND_ANGLE_TURN;
				break;
		}
	}
	
	public void selectSegmentForTuningLeftSwitch() {
		switch(currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				tuningVarValue = LeftSwitchVal.FIRST_DRIVE;
				break;
			case AutoSegments.SECOND_SEGMENT:
				tuningVarValue = LeftSwitchVal.FIRST_ANGLE_TURN;
				break;
			case AutoSegments.THIRD_SEGMENT:
				tuningVarValue = LeftSwitchVal.SECOND_DRIVE;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				tuningVarValue = LeftSwitchVal.SECOND_ANGLE_TURN;
				break;
			case AutoSegments.FIFTH_SEGMENT:
				tuningVarValue = LeftSwitchVal.THIRD_DRIVE;
				break;
		}
	}
	
	public void selectSegmentForTuningRightSwitch() {
		switch(currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				tuningVarValue = RightSwitchVal.FIRST_DRIVE;
				break;
			case AutoSegments.SECOND_SEGMENT:
				tuningVarValue = RightSwitchVal.FIRST_ANGLE_TURN;
				break;
			case AutoSegments.THIRD_SEGMENT:
				tuningVarValue = RightSwitchVal.SECOND_DRIVE;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				tuningVarValue = RightSwitchVal.SECOND_ANGLE_TURN;
				break;
			case AutoSegments.FIFTH_SEGMENT:
				tuningVarValue = RightSwitchVal.THIRD_DRIVE;
				break;
		}
	}
	
	public void setSegmentForTuning() {
		switch (currentlyTuningProgram) {
			case AutoPrograms.CENTER_LEFT_SWITCH:
				setSegmentValueCenterLeft();
				break;
			case AutoPrograms.CENTER_RIGHT_SWITCH:
				setSegmentValueCenterRight();
				break;
			case AutoPrograms.LEFT_SCALE:
				setSegmentValueLeftScale();
				break;
			case AutoPrograms.RIGHT_SCALE:
				setSegmentValueRightScale();
				break;
			case AutoPrograms.LEFT_SWITCH:
				setSegmentValueLeftSwitch();
				break;
			case AutoPrograms.RIGHT_SWITCH:
				setSegmentValueRightSwitch();
				break;
		}
	}
	
	public void setSegmentValueCenterRight() {
		switch (currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				CenterRightSwitch.FIRST_DRIVE = tuningVarValue;
				break;
			case AutoSegments.SECOND_SEGMENT:
				CenterRightSwitch.FIRST_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.THIRD_SEGMENT:
				CenterRightSwitch.SECOND_DRIVE = tuningVarValue;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				CenterRightSwitch.SECOND_ANGLE_TURN = tuningVarValue;
				break;
		}
	}
	
	public void setSegmentValueCenterLeft() {
		switch (currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				CenterLeftSwitch.FIRST_DRIVE = tuningVarValue;
				break;
			case AutoSegments.SECOND_SEGMENT:
				CenterLeftSwitch.FIRST_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.THIRD_SEGMENT:
				CenterLeftSwitch.SECOND_DRIVE = tuningVarValue;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				CenterLeftSwitch.SECOND_ANGLE_TURN = tuningVarValue;
				break;
		}
	}
	
	public void setSegmentValueRightScale() {
		switch (currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				RightScaleVal.FIRST_DRIVE = tuningVarValue;
				break;
			case AutoSegments.SECOND_SEGMENT:
				RightScaleVal.FIRST_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.THIRD_SEGMENT:
				RightScaleVal.SECOND_DRIVE = tuningVarValue;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				RightScaleVal.SECOND_ANGLE_TURN = tuningVarValue;
				break;
		}
	}
	
	public void setSegmentValueLeftScale() {
		switch (currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				LeftScaleVal.FIRST_DRIVE = tuningVarValue;
				break;
			case AutoSegments.SECOND_SEGMENT:
				LeftScaleVal.FIRST_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.THIRD_SEGMENT:
				LeftScaleVal.SECOND_DRIVE = tuningVarValue;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				LeftScaleVal.SECOND_ANGLE_TURN = tuningVarValue;
				break;
		}
	}
	
	public void setSegmentValueRightSwitch() {
		switch (currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				RightSwitchVal.FIRST_DRIVE = tuningVarValue;
				break;
			case AutoSegments.SECOND_SEGMENT:
				RightSwitchVal.FIRST_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.THIRD_SEGMENT:
				RightSwitchVal.SECOND_DRIVE = tuningVarValue;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				RightSwitchVal.SECOND_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.FIFTH_SEGMENT:
				RightSwitchVal.THIRD_DRIVE = tuningVarValue;
				break;
		}
	}
	
	public void setSegmentValueLeftSwitch() {
		switch (currentlyTuningSegment) {
			case AutoSegments.FIRST_SEGMENT:
				LeftSwitchVal.FIRST_DRIVE = tuningVarValue;
				break;
			case AutoSegments.SECOND_SEGMENT:
				LeftSwitchVal.FIRST_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.THIRD_SEGMENT:
				LeftSwitchVal.SECOND_DRIVE = tuningVarValue;
				break;
			case AutoSegments.FOURTH_SEGMENT:
				LeftSwitchVal.SECOND_ANGLE_TURN = tuningVarValue;
				break;
			case AutoSegments.FIFTH_SEGMENT:
				LeftSwitchVal.THIRD_DRIVE = tuningVarValue;
				break;
		}
	}
	
	public void startProgram() {
		switch(currentlyTuningProgram) {
			case AutoPrograms.CENTER_LEFT_SWITCH:
				new CenterSwitchLeft().start();
				break;
			case AutoPrograms.CENTER_RIGHT_SWITCH:
				new CenterSwitchRight().start();
				break;
			case AutoPrograms.RIGHT_SCALE:
				new RightScale().start();
				break;
			case AutoPrograms.LEFT_SCALE:
				new LeftScale().start();
				break;
			case AutoPrograms.RIGHT_SWITCH:
				new RightSwitch().start();
				break;
			case AutoPrograms.LEFT_SWITCH:
				new LeftSwitch().start();
				break;
		}
	}
	
	public String getSegmentNameCenterRight() {
		switch (currentlyTuningSegment) {
		case AutoSegments.FIRST_SEGMENT:
			return "First Drive";
		case AutoSegments.SECOND_SEGMENT:
			return "First Angle Turn";
		case AutoSegments.THIRD_SEGMENT:
			return "Second Drive";
		case AutoSegments.FOURTH_SEGMENT:
			return "Second Angle Turn";
		default:
			return "Found Nothing";
		}
	}
	
	public String getSegmentNameCenterLeft() {
		switch (currentlyTuningSegment) {
		case AutoSegments.FIRST_SEGMENT:
			return "First Drive";
		case AutoSegments.SECOND_SEGMENT:
			return "First Angle Turn";
		case AutoSegments.THIRD_SEGMENT:
			return "Second Drive";
		case AutoSegments.FOURTH_SEGMENT:
			return "Second Angle Turn";
		default:
			return "Found Nothing";
		}
	}
	
	public String getSegmentNameRightScale() {
		switch (currentlyTuningSegment) {
		case AutoSegments.FIRST_SEGMENT:
			return "First Drive";
		case AutoSegments.SECOND_SEGMENT:
			return "First Angle Turn";
		case AutoSegments.THIRD_SEGMENT:
			return "Second Drive";
		case AutoSegments.FOURTH_SEGMENT:
			return "Second Angle Turn";
		default:
			return "Found Nothing";
		}
	}
	
	public String getSegmentNameLeftScale() {
		switch (currentlyTuningSegment) {
		case AutoSegments.FIRST_SEGMENT:
			return "First Drive";
		case AutoSegments.SECOND_SEGMENT:
			return "First Angle Turn";
		case AutoSegments.THIRD_SEGMENT:
			return "Second Drive";
		case AutoSegments.FOURTH_SEGMENT:
			return "Second Angle Turn";
		default:
			return "Found Nothing";
		}
	}
	
	public String getSegmentNameRightSwitch() {
		switch (currentlyTuningSegment) {
		case AutoSegments.FIRST_SEGMENT:
			return "First Drive";
		case AutoSegments.SECOND_SEGMENT:
			return "First Angle Turn";
		case AutoSegments.THIRD_SEGMENT:
			return "Second Drive";
		case AutoSegments.FOURTH_SEGMENT:
			return "Second Angle Turn";
		case AutoSegments.FIFTH_SEGMENT:
			return "Third Drive";
		default:
			return "Found Nothing";
		}
	}
	
	public String getSegmentNameLeftSwitch() {
		switch (currentlyTuningSegment) {
		case AutoSegments.FIRST_SEGMENT:
			return "First Drive";
		case AutoSegments.SECOND_SEGMENT:
			return "First Angle Turn";
		case AutoSegments.THIRD_SEGMENT:
			return "Second Drive";
		case AutoSegments.FOURTH_SEGMENT:
			return "Second Angle Turn";
		case AutoSegments.FIFTH_SEGMENT:
			return "Third Drive";
		default:
			return "Found Nothing";
		}
	}
	
	public String getSegmentName() {
		switch(currentlyTuningProgram) {
		case AutoPrograms.CENTER_LEFT_SWITCH:
			return getSegmentNameCenterLeft();
		case AutoPrograms.CENTER_RIGHT_SWITCH:
			return getSegmentNameCenterRight();
		case AutoPrograms.RIGHT_SCALE:
			return getSegmentNameRightScale();
		case AutoPrograms.LEFT_SCALE:
			return getSegmentNameLeftScale();
		case AutoPrograms.RIGHT_SWITCH:
			return getSegmentNameRightSwitch();
		case AutoPrograms.LEFT_SWITCH:
			return getSegmentNameLeftSwitch();
		default:
			return "Found Nothing (Segment)";
		}
	}
	public String getProgramName() {
		switch(currentlyTuningProgram) {
		case AutoPrograms.CENTER_LEFT_SWITCH:
			return "Center Left Switch";
		case AutoPrograms.CENTER_RIGHT_SWITCH:
			return "Center Right Switch";
		case AutoPrograms.RIGHT_SCALE:
			return "Right Scale";
		case AutoPrograms.LEFT_SCALE:
			return "Left Scale";
		case AutoPrograms.RIGHT_SWITCH:
			return "Right Switch";
		case AutoPrograms.LEFT_SWITCH:
			return "Left Switch";
		default:
			return "Found Nothing (Program)";
		}
	}
	
	public void SmartDashboard() {
		SmartDashboard.putNumber("Tuning Amount: ", tuningIncrement);
		SmartDashboard.putString("Current Program", getProgramName());
		SmartDashboard.putString("Current Segment: ", getSegmentName());
		SmartDashboard.putNumber("Segment Val", tuningVarValue);
		SmartDashboard.putNumber("Gyro Val", Robot.gyro.getGyroAngle());
	}
}
