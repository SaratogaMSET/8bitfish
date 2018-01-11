package org.usfirst.frc.team649.robot;

//constants for ports other constants will be placed in respective subsystems
public class RobotMap {
	public static final int BUTTON_BOARD = 0;
	public static final int DRIVE_JOYSTICK_HORIZONTAL = 2;
	public static final int DRIVE_JOYSTICK_VERTICAL = 1;
	
	public static class Drivetrain {
		//change before first use
		public static final int[] LEFT_SIDE_ENCODER = { 4, 5 };
		public static final int[] RIGHT_SIDE_ENCODER = { 6, 7 };
		// FR,BR,BL,FL
		public static final int[] MOTOR_PORTS = { 9, 14, 5, 16 };
		public static final int[] RIGHT_DRIVE_SOL = { 2, 0, 1 };
		public static final int[] LEFT_DRIVE_SOL = { 2, 2, 3 };
	}
	
}
