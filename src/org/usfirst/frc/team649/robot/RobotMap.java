package org.usfirst.frc.team649.robot;

//constants for ports other constants will be placed in respective subsystems
public class RobotMap {
	public static final int BUTTON_BOARD = 4;
	public static final int BUTTON_BOARD_2 = 3;
	public static final int OPERATOR_JOYSTICK = 2;
	public static final int DRIVE_JOYSTICK_HORIZONTAL = 1;
	public static final int DRIVE_JOYSTICK_VERTICAL = 0;
	public static final int POWER_DISTRIBUTION_PANEL = 0;

	
	public static class Drivetrain {
		// FL,BL,BR,FR
		public static final int[] MOTOR_PORTS = {12, 13, 20, 19}; //20 on final bot
		public static final int[] RIGHT_DRIVE_SOL = {4,4,1}; //{4, 4, 1}; // 4,4,5
		public static final int[] LEFT_DRIVE_SOL = { 3, 2, 3 };
	}
	public static class Lift {
		//mag srx encoder on 11
		public static final int LEFT_WINCH_MOTOR = 10;
		public static final int RIGHT_WINCH_MOTOR = 16;
		public static final int LEFT_WINCH_SECOND_MOTOR = 11;
		
		public static final int CARRIAGE_HAL_BOT_RIGHT = 0;
		public static final int CARRIAGE_HAL_BOT_LEFT = 2;

		public static final int CARRIAGE_HAL_TOP_RIGHT = 1;
		public static final int CARRIAGE_HAL_TOP_LEFT = 3;

		public static final int SECOND_STAGE_HAL_BOT_RIGHT = 7;
		public static final int SECOND_STAGE_HAL_BOT_LEFT = 4;

		public static final int SECOND_STAGE_HAL_TOP_RIGHT = 6;
		public static final int SECOND_STAGE_HAL_TOP_LEFT = 5;
		
	}
	public static class Arm {
		public static final int BOTTOM_ARM_MOTOR = 8;
		public static final int TOP_ARM_MOTOR = 17;
		public static final int INFRARED_SENSOR = 8;
		public static final int[] ARM_BRAKE = {4,6,7}; //final 4,6,7  4,0,7
		public static final int ARM_HAL_FRONT = 9;
		public static final int ARM_HAL_REAR = 10;
	}
	public static class AutoSwitches{
		public static final int[] Auto_Switches = {0,1,2,3,4};
	}
	public static class Intake {
		public static final int LEFT_INTAKE_MOTOR = 9;
		public static final int RIGHT_INTAKE_MOTOR = 15;
		public static final int[] INTAKE_SOL = {3,0,1}; //60
		public static final int[] SECOND_SOL = {4,2,3}; //30
		
		//when trying to intake closed, close both
		//while trying to clamp cube run only 60
		//open run just 30
		
	}
	public static class Camera{
        public static final boolean practiceBot = false;
        public static final String axisPort = "10.6.49.7";
        public static final String axisName = "tanAxisCamera";
        public static final int axisResWidth = 160;
        public static final int axisResHeight = 120;
        public static final int axisFPS = 20;
    }
}
