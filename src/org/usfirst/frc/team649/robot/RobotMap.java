package org.usfirst.frc.team649.robot;

//constants for ports other constants will be placed in respective subsystems
public class RobotMap {
	public static final int BUTTON_BOARD = 4;
	public static final int OPERATOR_JOYSTICK = 2;
	public static final int DRIVE_JOYSTICK_HORIZONTAL = 1;
	public static final int DRIVE_JOYSTICK_VERTICAL = 0;
	
	public static class Drivetrain {
		//change before first use
		public static final int[] LEFT_SIDE_ENCODER = { 4, 5 };
		public static final int[] RIGHT_SIDE_ENCODER = { 6, 7 };
		// FL,BL,BR,FR
		public static final int[] MOTOR_PORTS = { 12, 13, 18, 19 };
		public static final int[] RIGHT_DRIVE_SOL = { 2, 0, 1 };
		public static final int[] LEFT_DRIVE_SOL = { 2, 2, 3 };
	}
	public static class Lift {
		//mag srx encoder on 11
		public static final int LEFT_WINCH_MOTOR = 10;
		public static final int RIGHT_WINCH_MOTOR = 16;
		public static final int ARM_HAL_BOT = 8;
		public static final int ARM_HAL_TOP = 0;
		public static final int SECOND_STAGE_HAL_BOT = 7;
		public static final int SECOND_STAGE_HAL_TOP = 9;
	}

	public static class Camera{
		public static final String axisPort = "10.6.49.15";
		public static final String axisName = "greenAxisCamera";
		public static final int axisResWidth = 320;
		public static final int axisResHeight = 480;
		public static final String axis2Port = "10.6.49.17";
		public static final String axis2Name = "blackAxisCamera";
		public static final int axisFPS = 30;
		public static final int axis2ResWidth = 320;
		public static final int axis2ResHeight = 480;
		public static final int axis2FPS = 30;
		public static final String axis3Port = "10.6.49.13";
		public static final String axis3Name = "whiteAxisCamera";
		public static final int axis3ResWidth = 320;
		public static final int axis3ResHeight = 480;
		public static final int axis3FPS = 30;
	}
}
