package org.usfirst.frc.team649.robot;

//constants for ports other constants will be placed in respective subsystems
public class RobotMap {
	public static final int BUTTON_BOARD = 4;
	public static final int DRIVE_JOYSTICK_HORIZONTAL = 1;
	public static final int DRIVE_JOYSTICK_VERTICAL = 0;
	
	public static class Drivetrain {
		//change before first use
		public static final int[] LEFT_SIDE_ENCODER = { 4, 5 };
		public static final int[] RIGHT_SIDE_ENCODER = { 6, 7 };
		// FR,BR,BL,FL
		public static final int[] MOTOR_PORTS = { 10, 11, 12, 13 };
		public static final int[] RIGHT_DRIVE_SOL = { 2, 0, 1 };
		public static final int[] LEFT_DRIVE_SOL = { 2, 2, 3 };
	}
	
}
