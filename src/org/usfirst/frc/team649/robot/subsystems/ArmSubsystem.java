package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ArmSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public static class ArmConstants{
		public static final double ARM_POWER = 0.6;
		public static final int RAW_ABS_TOL = 10;
	}
	public static class ArmStateConstants{
		public static final int HEADING_INTAKE_FRONT = 1;
		public static final int INTAKE_FRONT = 2;
		public static final int HEADING_EXCHANGE_FRONT = 3;
		public static final int EXCHANGE_FRONT = 4;
		public static final int HEADING_STORE_FRONT = 5;
		public static final int STORE_FRONT = 6;
		public static final int HEADING_MID_DROP_FRONT = 7;
		public static final int MID_DROP_FRONT = 8;
		public static final int HEADING_HIGH_DROP_FRONT = 9;
		public static final int HIGH_DROP_FRONT = 10;
		public static final int HEADING_CUSTOM_UP = 13;
		public static final int CUSTOM = 12;
		public static final int HEADING_SWITCH_FRONT = 14;
		public static final int SWITCH_FRONT = 15;
		public static final int HEADING_CUSTOM_DOWN = 11;
		public static final int HEADING_INTAKE_REAR = -1;
		public static final int INTAKE_REAR = -2;
		public static final int HEADING_EXCHANGE_REAR = -3;
		public static final int EXCHANGE_REAR = -4;
		public static final int HEADING_STORE_REAR = -5;
		public static final int STORE_REAR = -6;
		public static final int HEADING_MID_DROP_REAR = -7;
		public static final int MID_DROP_REAR = -8;
		public static final int HEADING_HIGH_DROP_REAR = -9;
		public static final int HIGH_DROP_REAR = -10;
		public static final int HEADING_SWITCH_REAR = -14;
		public static final int SWITCH_REAR = -15;
	}
	public static class ArmEncoderConstants{
		public static final int INTAKE_FRONT = 0;
		public static final int INTAKE_REAR = -4350; //4220
		public static final int SWITCH_FRONT = -1450;
		public static final int SWITCH_REAR = -2900;
		public static final int EXCHANGE_FRONT = -400;
		public static final int EXCHANGE_REAR = -3950;
		public static final int MID_DROP_FRONT = -550;
		public static final int MID_DROP_REAR = -3800;
		public static final int HIGH_DROP_FRONT = -1000;
		public static final int HIGH_DROP_REAR = -3350;
		public static final int STORE_FRONT = -1625;
		public static final int STORE_REAR = -2725;
		public static final int ADJ = 150;
		public static final int MID = (INTAKE_FRONT + INTAKE_REAR)/2;
	}
	
	DigitalInput infraredSensor;
	public TalonSRX bottomMotor;
	public TalonSRX topMotor;
	public double lastVal;
	public Timer time;
	public DoubleSolenoid armBrake;
	public DigitalInput frontHal,rearHal;
	public ArmSubsystem() {
		bottomMotor = new TalonSRX(RobotMap.Arm.BOTTOM_ARM_MOTOR);
		bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Robot.timeoutMs);
		topMotor = new TalonSRX(RobotMap.Arm.TOP_ARM_MOTOR);
		
		infraredSensor = new DigitalInput(RobotMap.Arm.INFRARED_SENSOR);
		
		bottomMotor.configNominalOutputForward(0, Robot.timeoutMs);
		bottomMotor.configNominalOutputReverse(0, Robot.timeoutMs);
		bottomMotor.configPeakOutputForward(1.0, Robot.timeoutMs);
		bottomMotor.configPeakOutputReverse(-1.0, Robot.timeoutMs);
		bottomMotor.setSensorPhase(true); // true on f bot
		bottomMotor.setInverted(true);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Robot.timeoutMs);
		
		frontHal = new DigitalInput(RobotMap.Arm.ARM_HAL_FRONT);
		rearHal = new DigitalInput(RobotMap.Arm.ARM_HAL_REAR);
		
		lastVal = 0;
		
		bottomMotor.configMotionAcceleration(650, Robot.timeoutMs);
		bottomMotor.configMotionCruiseVelocity(700, Robot.timeoutMs);

		bottomMotor.config_kP(0, 1, Robot.timeoutMs);
		bottomMotor.config_kI(0, 0, Robot.timeoutMs);
		bottomMotor.config_kD(0, 0, Robot.timeoutMs);
		bottomMotor.config_kF(0, 1.25, Robot.timeoutMs);
		bottomMotor.selectProfileSlot(0, 0);
		
		bottomMotor.setNeutralMode(NeutralMode.Brake);
		topMotor.setNeutralMode(NeutralMode.Brake);

		topMotor.setInverted(false);
		topMotor.set(ControlMode.Follower, RobotMap.Arm.BOTTOM_ARM_MOTOR);
		armBrake = new DoubleSolenoid(RobotMap.Arm.ARM_BRAKE[0],RobotMap.Arm.ARM_BRAKE[1],RobotMap.Arm.ARM_BRAKE[2]);
	}
	
	public void setArm(double power){
		bottomMotor.set(ControlMode.PercentOutput, -power);
	}
	public boolean getInfraredSensor() {
		return !infraredSensor.get();
	}
	public boolean getArmHalZeroFront(){
		return !frontHal.get();
	}
	public boolean getArmHalZeroBack(){
		return !rearHal.get();
	}
	public void setArmBrake(boolean isEngaged){
		armBrake.set(isEngaged ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
	}
	public int getArmRaw() {
		return bottomMotor.getSelectedSensorPosition(0);
	}

	public double getVel() {
		lastVal = bottomMotor.getSensorCollection().getQuadratureVelocity();
		return lastVal;
	}
	
	public double getAccel() {
		return (bottomMotor.getSensorCollection().getQuadratureVelocity() - lastVal);
	}
	
	public void setEncoder(int position) {
		bottomMotor.setSelectedSensorPosition(position, 0, Robot.timeoutMs);
	}
	

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

