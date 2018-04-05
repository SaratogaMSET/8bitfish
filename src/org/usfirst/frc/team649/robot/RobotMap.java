package org.usfirst.frc.team649.robot;

//constants for ports other constants will be placed in respective subsystems
public class RobotMap {
	public static final int BUTTON_BOARD = 4;
	public static final int BUTTON_BOARD_2 = 3;
	public static final int OPERATOR_JOYSTICK = 2;
	public static final int DRIVE_JOYSTICK_HORIZONTAL = 1;
	public static final int DRIVE_JOYSTICK_VERTICAL = 0;

	
	public static class Drivetrain {
		//change before first use
//		public static final int[] LEFT_SIDE_ENCODER = { 4, 5 };
//		public static final int[] RIGHT_SIDE_ENCODER = { 6, 7 };
		// FL,BL,BR,FR
		public static final int[] MOTOR_PORTS = {12, 13, 18, 19}; //18-->20
		public static final int[] RIGHT_DRIVE_SOL = {4, 4, 1}; // 4,4,5
		public static final int[] LEFT_DRIVE_SOL = { 3,2, 3 };
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
		public static final int[] ARM_BRAKE = {4,6,7}; // 4,6,7  4,0,7
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
        public static final int axisResWidth = 320;
        public static final int axisResHeight = 240;
        public static final int axisFPS = 30;
//        public static final String axis2Port = "10.6.49.17";
//        public static final String axis2Name = "blackAxisCamera";
//        public static final int axis2ResWidth = 320;
//        public static final int axis2ResHeight = 480;
//        public static final int axis2FPS = 30;
//        public static final String axis3Port = "10.6.49.13";
//        public static final String axis3Name = "whiteAxisCamera";
//        public static final int axis3ResWidth = 320;
//        public static final int axis3ResHeight = 480;
//        public static final int axis3FPS = 30;
//        public static final String axis4Port = "10.6.49.9";
//        public static final String axis4Name = "greenAxisCamera";
//        public static final int axis4ResWidth = 320;
//        public static final int axis4ResHeight = 480;
//        public static final int axis4FPS = 30;
//        public static final String axis5Port = "10.6.49.11";
//        public static final String axis5Name = "greenAxisCamera";
//        public static final int axis5ResWidth = 320;
//        public static final int axis5ResHeight = 480;
////        public static final int axis5FPS = 30;
//        public static final String axis6Port = "10.6.49.7";
//        public static final String axis6Name = "tanAxisCamera";
//        public static final int axis6ResWidth = 320;
//        public static final int axis6ResHeight = 480;
//        public static final int axis6FPS = 30;
    }
	public class Hang
	{
		public static final int hangport = 0;
	}
}
