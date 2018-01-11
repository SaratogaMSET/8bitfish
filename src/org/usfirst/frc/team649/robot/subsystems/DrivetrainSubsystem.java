package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DrivetrainSubsystem extends Subsystem {
	
	//if it is actualy running not if it should be in that mode
	private static boolean isLeftVPid;
	private static boolean isRightVPid;
	private static boolean isHighGear;

	//don't save pid stuff here this is for distances and angles etc
    public static class AutoConstants{
    	
    }
    
    public static class AutoPIDConstants{
    	public static final double PID_ABS_TOLERANCE = 0;
    	public static final double k_P = 0.0;
		public static final double k_I = 0.0;
		public static final double k_D = 0.0;
		public static final double k_F = 0.0;
		public static final double DISTANCE_PER_PULSE_LOW = 0;
		public static final double DISTANCE_PER_PULSE_HIGH = 0; 	
    }
    
    public static class VPIDConstants{
    	public static final double MAX_RPM_LOW = 0;
    	public static final double MAX_RPM_HIGH = 0;
    	public static final int DISTANCE_PER_REV = 2048;
    	//high end is when to shift up
    	//low end is when to shift back down
    	//might need to add something for turning we will see
    	public static final double SHIFT_LOW_END = 0;
    	public static final double SHIFT_HIGH_END = 0;
    	public static final double k_P_LOW = 0.0;
		public static final double k_I_LOW = 0.0;
		public static final double k_D_LOW = 0.0;
		public static final double k_F_LOW = 0.0;
		public static final double k_P_HIGH = 0.0;
		public static final double k_I_HIGH = 0.0;
		public static final double k_D_HIGH = 0.0;
		public static final double k_F_HIGH = 0.0;
    }
    
    public Encoder leftEncoder, rightEncoder;

    public TalonSRX[] motors;
	public DoubleSolenoid driveSolLeft, driveSolRight;
	
	public DrivetrainSubsystem() {
		leftEncoder = new Encoder(RobotMap.Drivetrain.LEFT_SIDE_ENCODER[0],RobotMap.Drivetrain.LEFT_SIDE_ENCODER[1],RobotMap.Drivetrain.LEFT_SIDE_ENCODER[2]);
		rightEncoder = new Encoder(RobotMap.Drivetrain.RIGHT_SIDE_ENCODER[0],RobotMap.Drivetrain.RIGHT_SIDE_ENCODER[1],RobotMap.Drivetrain.RIGHT_SIDE_ENCODER[2]);
		for (int i = 0; i < motors.length; i++) {
			motors[i] = new TalonSRX(RobotMap.Drivetrain.MOTOR_PORTS[i]);
		}
		motors[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		motors[0].setSensorPhase(false);
		motors[0].configNominalOutputForward(0,10);
		motors[0].configNominalOutputReverse(0, 10);
		motors[0].configPeakOutputForward(1, 10);
		motors[0].configPeakOutputReverse(-1, 10);
//		motors[0].configEncoderCodesPerRev(VPIDConstants.DISTANCE_PER_REV);
//		motors[1].changeControlMode(ControlMode.Follower);
		motors[1].set(ControlMode.Follower,RobotMap.Drivetrain.MOTOR_PORTS[0]);
		motors[2].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		motors[2].setSensorPhase(false);
		motors[2].configNominalOutputForward(0,10);
		motors[2].configNominalOutputReverse(0, 10);
		motors[2].configPeakOutputForward(1, 10);
		motors[2].configPeakOutputReverse(-1, 10);	
//		motors[2].configEncoderCodesPerRev(VPIDConstants.DISTANCE_PER_REV);
		motors[3].set(ControlMode.Follower,RobotMap.Drivetrain.MOTOR_PORTS[2]);
		isLeftVPid = false;
		isRightVPid = false;
		isHighGear = false;
	}
	//changes the drivetrain between vbus and vpid 
	private void changeDrivetrainModesLeft(boolean isVPid){
		if(isVPid){
//			motors[0].changeControlMode(TalonControlMode.Speed);
			isLeftVPid = true;
		}else{
			isLeftVPid = false;
//			motors[0].changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	private void changeDrivetrainModesRight(boolean isVPid){
		if(isVPid){
//			motors[2].changeControlMode(TalonControlMode.Speed);
			isRightVPid = true;
		}else{
			isRightVPid = false;
//			motors[2].changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	public void changeBrakeCoast(boolean isBrake){
		if(isBrake){
			motors[0].setNeutralMode(NeutralMode.Brake);
			motors[1].setNeutralMode(NeutralMode.Brake);
			motors[2].setNeutralMode(NeutralMode.Brake);
			motors[3].setNeutralMode(NeutralMode.Brake);
		}else{
			motors[0].setNeutralMode(NeutralMode.Coast);
			motors[1].setNeutralMode(NeutralMode.Coast);
			motors[2].setNeutralMode(NeutralMode.Coast);
			motors[3].setNeutralMode(NeutralMode.Coast);
		}
	}
	
	public void shift(boolean isHigh){
		isHighGear = isHigh;
		driveSolLeft.set(isHigh ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
		driveSolRight.set(isHigh ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
		if(isHigh){
			leftEncoder.setDistancePerPulse(AutoPIDConstants.DISTANCE_PER_PULSE_HIGH);
			rightEncoder.setDistancePerPulse(AutoPIDConstants.DISTANCE_PER_PULSE_HIGH);
			if(isLeftVPid){
				motors[0].config_kF(0,VPIDConstants.k_F_HIGH,10); 
				motors[0].config_kP(0,VPIDConstants.k_P_HIGH,10);
				motors[0].config_kI(0,VPIDConstants.k_I_HIGH,10);
				motors[0].config_kD(0,VPIDConstants.k_D_HIGH,10); 
			}
			if(isRightVPid){
				motors[2].config_kF(0,VPIDConstants.k_F_HIGH,10); 
				motors[2].config_kP(0,VPIDConstants.k_P_HIGH,10);
				motors[2].config_kI(0,VPIDConstants.k_I_HIGH,10);
				motors[2].config_kD(0,VPIDConstants.k_D_HIGH,10);
			}
		}else{
			leftEncoder.setDistancePerPulse(AutoPIDConstants.DISTANCE_PER_PULSE_LOW);
			rightEncoder.setDistancePerPulse(AutoPIDConstants.DISTANCE_PER_PULSE_LOW);
			if(isLeftVPid){
				motors[0].config_kF(0,VPIDConstants.k_F_LOW,10); 
				motors[0].config_kP(0,VPIDConstants.k_P_LOW,10);
				motors[0].config_kI(0,VPIDConstants.k_I_LOW,10);
				motors[0].config_kD(0,VPIDConstants.k_D_LOW,10);
			}else{
				motors[2].config_kF(0,VPIDConstants.k_F_LOW,10); 
				motors[2].config_kP(0,VPIDConstants.k_P_LOW,10);
				motors[2].config_kI(0,VPIDConstants.k_I_LOW,10);
				motors[2].config_kD(0,VPIDConstants.k_D_LOW,10);
			}
		}
	}
	
	public void autoShift(){
		double speed = Math.abs(leftEncoder.getRate()) + Math.abs(rightEncoder.getRate());
		if(isHighGear && speed < VPIDConstants.MAX_RPM_LOW){
			shift(true);
		}else if(!isHighGear && speed > VPIDConstants.MAX_RPM_HIGH){
			shift(false);
		}
	}
	
	public void driveFwdRotate(double fwd, double roti, boolean isVBus){
		double rot = roti * roti;
		if(roti < 0){
			rot = -1*rot;
		}
		double left = fwd + rot, right = fwd - rot;
		double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
		left /= max;
		right /= max;
		if(isVBus){
			changeDrivetrainModesLeft(false);
			changeDrivetrainModesRight(false);
			motors[0].set(ControlMode.PercentOutput,left);
			motors[2].set(ControlMode.PercentOutput,-right);
		}else{
			rawDriveVelPidLeft(left);
			rawDriveVelPidRight(-right);
		}
	}
	
	private void rawDriveVelPidLeft(double left){
		if(Math.abs(left) > 0.05){
			changeDrivetrainModesLeft(true);
			shift(isHighGear);
			if(isHighGear){
				motors[0].set(ControlMode.Velocity, left * VPIDConstants.MAX_RPM_HIGH); 
			}else{
				motors[0].set(ControlMode.Velocity, left * VPIDConstants.MAX_RPM_LOW); 
			}
		}else{
			changeDrivetrainModesLeft(false);
			motors[0].set(ControlMode.PercentOutput,0);
		}
	}
	private void rawDriveVelPidRight(double right){
		if(Math.abs(right) > 0.05){
			changeDrivetrainModesRight(true);
			shift(isHighGear);
			if(isHighGear){
				motors[2].set(ControlMode.Velocity,-right * VPIDConstants.MAX_RPM_HIGH); 
			}else{
				motors[2].set(ControlMode.Velocity,-right * VPIDConstants.MAX_RPM_LOW); 
			}
		}else{
			changeDrivetrainModesRight(false);
			motors[2].set(ControlMode.PercentOutput,0);
		}
	}
	
    public void initDefaultCommand() {
    	
    }
}

