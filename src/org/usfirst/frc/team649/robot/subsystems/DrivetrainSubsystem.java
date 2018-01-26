package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.subsystems.GyroSubsystem.GyroPIDConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DrivetrainSubsystem extends PIDSubsystem {
	
	//if it is actualy running not if it should be in that mode
	private static boolean isLeftVPid;
	private static boolean isRightVPid;
	private static boolean isHighGear;

	//don't save pid stuff here this is for distances and angles etc
    public static class AutoConstants{
    	
    }
    
    public static class AutoPIDConstants{
    	public static final double PID_ABS_TOLERANCE = 2;
    	public static double k_P = 0.2;
		public static double k_I = 0.0;
		public static double k_D = 0.1;
		public static double k_F = 0.0;
		public static final double DISTANCE_PER_PULSE_LOW = 0;
		public static final double DISTANCE_PER_PULSE_HIGH = 0; 	
    }
    
    public static class DrivePIDConstants {
    	public static final double GYRO_ABS_TOLERANCE = 0;
    	public static final double k_p = 0.0;
    	public static final double k_i = 0.0;
    	public static final double k_d = 0.0;
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
		public static final double Y_COMPONENT_EXP = 1.32;
		public static final double X_COMPONENT_HIGH_EXP = 1.75;
		public static final double X_COMPONENT_LOW_EXP = 3;
    }
    
    
    public Encoder leftEncoder, rightEncoder;
    
    public double drivePIDOutput;
    public TalonSRX[] motors;
	public DoubleSolenoid driveSolLeft, driveSolRight;
	
	public DrivetrainSubsystem() {
		super("Drivetrain Subsystem", AutoPIDConstants.k_P, AutoPIDConstants.k_I, AutoPIDConstants.k_D);
		
		drivePIDOutput = 0;
		
//		leftEncoder = new Encoder(RobotMap.Drivetrain.LEFT_SIDE_ENCODER[0],RobotMap.Drivetrain.LEFT_SIDE_ENCODER[1]);
//		leftEncoder.setDistancePerPulse(4.00 * Math.PI / 2048.0 * 14 / 60);
//		rightEncoder = new Encoder(RobotMap.Drivetrain.RIGHT_SIDE_ENCODER[0],RobotMap.Drivetrain.RIGHT_SIDE_ENCODER[1]);
		motors = new TalonSRX[4];
		for (int i = 0; i < motors.length; i++) {
			motors[i] = new TalonSRX(RobotMap.Drivetrain.MOTOR_PORTS[i]);
		}
		
		motors[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
		motors[0].setSensorPhase(false);
		motors[0].configNominalOutputForward(0, 30);
		motors[0].configNominalOutputReverse(0, 30);
		motors[0].configPeakOutputForward(1, 30);
		motors[0].configPeakOutputReverse(-1, 30);
		
		motors[1].configNominalOutputForward(0, 30);
		motors[1].configNominalOutputReverse(0, 30);
		motors[1].configPeakOutputForward(1, 30);
		motors[1].configPeakOutputReverse(-1, 30);
		
//		motors[0].configEncoderCodesPerRev(VPIDConstants.DISTANCE_PER_REV);
//		motors[1].changeControlMode(ControlMode.Follower);
		motors[1].set(ControlMode.Follower,RobotMap.Drivetrain.MOTOR_PORTS[0]);
		motors[2].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
		motors[2].setSensorPhase(false);
		motors[2].configNominalOutputForward(0, 30);
		motors[2].configNominalOutputReverse(0, 30);
		motors[2].configPeakOutputForward(1, 30);
		motors[2].configPeakOutputReverse(-1, 30);

		
		motors[3].configNominalOutputForward(0, 30);
		motors[3].configNominalOutputReverse(0, 30);
		motors[3].configPeakOutputForward(1, 30);
		motors[3].configPeakOutputReverse(-1, 30);
		
//		motors[2].configEncoderCodesPerRev(VPIDConstants.DISTANCE_PER_REV);
		motors[3].set(ControlMode.Follower,RobotMap.Drivetrain.MOTOR_PORTS[2]);
		isLeftVPid = false;
		isRightVPid = false;
		isHighGear = true;
		this.getPIDController().setAbsoluteTolerance(AutoPIDConstants.PID_ABS_TOLERANCE);
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
				motors[0].config_kF(0,VPIDConstants.k_F_HIGH,30); 
				motors[0].config_kP(0,VPIDConstants.k_P_HIGH,30);
				motors[0].config_kI(0,VPIDConstants.k_I_HIGH,30);
				motors[0].config_kD(0,VPIDConstants.k_D_HIGH,30); 
			}
			if(isRightVPid){
				motors[2].config_kF(0,VPIDConstants.k_F_HIGH,30); 
				motors[2].config_kP(0,VPIDConstants.k_P_HIGH,30);
				motors[2].config_kI(0,VPIDConstants.k_I_HIGH,30);
				motors[2].config_kD(0,VPIDConstants.k_D_HIGH,30);
			}
		}else{
			leftEncoder.setDistancePerPulse(AutoPIDConstants.DISTANCE_PER_PULSE_LOW);
			rightEncoder.setDistancePerPulse(AutoPIDConstants.DISTANCE_PER_PULSE_LOW);
			if(isLeftVPid){
				motors[0].config_kF(0,VPIDConstants.k_F_LOW,30); 
				motors[0].config_kP(0,VPIDConstants.k_P_LOW,30);
				motors[0].config_kI(0,VPIDConstants.k_I_LOW,30);
				motors[0].config_kD(0,VPIDConstants.k_D_LOW,30);
			}else{
				motors[2].config_kF(0,VPIDConstants.k_F_LOW,30); 
				motors[2].config_kP(0,VPIDConstants.k_P_LOW,30);
				motors[2].config_kI(0,VPIDConstants.k_I_LOW,30);
				motors[2].config_kD(0,VPIDConstants.k_D_LOW,30);
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
	
	public void rawDrive(double left, double right) {
		motors[0].set(ControlMode.PercentOutput, left);
		motors[1].set(ControlMode.Follower, RobotMap.Drivetrain.MOTOR_PORTS[0]);
		motors[2].set(ControlMode.PercentOutput, -right);
		motors[3].set(ControlMode.Follower, RobotMap.Drivetrain.MOTOR_PORTS[2]);
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
	
	public double getTalonDistanceLeft() {
		double encPos = (double) motors[0].getSensorCollection().getQuadraturePosition();
		if (!isHighGear) {
			return ((encPos)/4096.0) *(14.0/60.0) * (5.0 * Math.PI) / 8.0 * 2;
		} else {
			return ((encPos)/4096.0) *(24.0/50) * (5.0 * Math.PI) / 8.0 * 2;
		}
	}
	
	public double getTalonDistanceRight() {
		double encPos = (double) motors[2].getSensorCollection().getQuadraturePosition();
		if (!isHighGear) {
			return ((encPos)/4096.0) *(14.0/60.0) * (5.0 * Math.PI) / 8.0 * 2;
		} else {
			return ((encPos)/4096.0) *(24.0/50) * (5.0 * Math.PI) / 8.0 * 2;
		}
	}
	public double getAvgTalonDistance() {
		return getTalonDistanceLeft() + getTalonDistanceRight()/ 2.0;
	}
	public void resetEncoders() {
		motors[0].getSensorCollection().setQuadraturePosition(0, 20);
		motors[2].getSensorCollection().setQuadraturePosition(0, 20);
//		leftEncoder.reset();
	}
    public void initDefaultCommand() {
    	
    }
    
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return getAvgTalonDistance();
	}
	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		//currentMotorPower = output;
		drivePIDOutput = output;
		
	}
	
	public double getDrivePIDOutput() {
		return drivePIDOutput;
	}
	
	public double getP() {
		return AutoPIDConstants.k_P;
	}
	public double getI() {
		return AutoPIDConstants.k_I;
	}
	public double getD() {
		return AutoPIDConstants.k_D;
	}
	public void setP(double d) {
		AutoPIDConstants.k_P += d;
	}
	public void setI(double d) {
		AutoPIDConstants.k_I += d;
	}
	public void setD(double d) {
		AutoPIDConstants.k_D += d;
	}
	
}

