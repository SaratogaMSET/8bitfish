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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainSubsystem extends PIDSubsystem {
	
	//if it is actualy running not if it should be in that mode
	private static boolean isLeftVPid;
	private static boolean isRightVPid;
	private static boolean isHighGear;
	public double wheelSize=5;
	public int scalingFactor=2;

	//don't save pid stuff here this is for distances and angles etc
    public static class AutoConstants{
    	
    	
    }
    
    public static class AutoPIDConstants{
    	public static final double PID_ABS_TOLERANCE = 1.0;
    	public static double k_P = 0.05;
		public static double k_I = 0.0;
		public static double k_D = 0.1;
		public static double k_F = 0.0;
		public static final double DISTANCE_PER_PULSE_LOW = 0;
		public static final double DISTANCE_PER_PULSE_HIGH = 0; 	
    }
    
    public TalonSRX[] motors;
	public DoubleSolenoid driveSolLeft, driveSolRight;
	
	public DrivetrainSubsystem() {
		super("Drivetrain Subsystem", AutoPIDConstants.k_P, AutoPIDConstants.k_I, AutoPIDConstants.k_D);
		
		driveSolLeft = new DoubleSolenoid(RobotMap.Drivetrain.LEFT_DRIVE_SOL[0], RobotMap.Drivetrain.LEFT_DRIVE_SOL[1], RobotMap.Drivetrain.LEFT_DRIVE_SOL[2]);
		driveSolRight = new DoubleSolenoid(RobotMap.Drivetrain.RIGHT_DRIVE_SOL[0], RobotMap.Drivetrain.RIGHT_DRIVE_SOL[1], RobotMap.Drivetrain.RIGHT_DRIVE_SOL[2]);
		
		motors = new TalonSRX[4];
		for (int i = 0; i < motors.length; i++) {
			motors[i] = new TalonSRX(RobotMap.Drivetrain.MOTOR_PORTS[i]);
		}
		
		motors[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Robot.timeoutMs);
		motors[0].setSensorPhase(false);
		motors[0].configNominalOutputForward(0, Robot.timeoutMs);
		motors[0].configNominalOutputReverse(0, Robot.timeoutMs);
		motors[0].configPeakOutputForward(1, Robot.timeoutMs);
		motors[0].configPeakOutputReverse(-1, Robot.timeoutMs);
		motors[0].configMotionCruiseVelocity(20000, Robot.timeoutMs);
		motors[0].configMotionAcceleration(11000, Robot.timeoutMs); //1360 actual
		motors[0].selectProfileSlot(0, 0);
		motors[0].config_kP(0, 1.5, Robot.timeoutMs); // 1.5
		motors[0].config_kI(0, 0.03, Robot.timeoutMs); // 0.03
		motors[0].config_kD(0, 1.25, Robot.timeoutMs); 
		motors[0].config_kF(0, 0.060176, Robot.timeoutMs);
		motors[0].setInverted(false);
		
		motors[1].configNominalOutputForward(0, Robot.timeoutMs);
		motors[1].configNominalOutputReverse(0, Robot.timeoutMs);
		motors[1].configPeakOutputForward(1, Robot.timeoutMs);
		motors[1].configPeakOutputReverse(-1, Robot.timeoutMs);
		motors[1].setInverted(false);
		motors[1].set(ControlMode.Follower,RobotMap.Drivetrain.MOTOR_PORTS[0]);
		
		motors[2].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Robot.timeoutMs);
		motors[2].setSensorPhase(false); //
		motors[2].setInverted(true);
		motors[2].configNominalOutputForward(0, Robot.timeoutMs);
		motors[2].configNominalOutputReverse(0, Robot.timeoutMs);
		motors[2].configPeakOutputForward(1, Robot.timeoutMs);
		motors[2].configPeakOutputReverse(-1, Robot.timeoutMs);
		motors[2].configMotionCruiseVelocity(20000, Robot.timeoutMs);
		motors[2].configMotionAcceleration(11000, Robot.timeoutMs); //1360 actual
		motors[2].selectProfileSlot(0, 0);
		motors[2].config_kP(0, 1.5 , Robot.timeoutMs);
		motors[2].config_kI(0, 0.03, Robot.timeoutMs);
		motors[2].config_kD(0, 1.25, Robot.timeoutMs);
		motors[2].config_kF(0, 0.060176, Robot.timeoutMs);

		
		motors[3].configNominalOutputForward(0, Robot.timeoutMs);
		motors[3].configNominalOutputReverse(0, Robot.timeoutMs);
		motors[3].configPeakOutputForward(1, Robot.timeoutMs);
		motors[3].configPeakOutputReverse(-1, Robot.timeoutMs);
		motors[3].set(ControlMode.Follower,RobotMap.Drivetrain.MOTOR_PORTS[2]);
		motors[3].setInverted(true);
		
		isLeftVPid = false;
		isRightVPid = false;
		isHighGear = true;
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
		SmartDashboard.putBoolean("Gear", isHigh);
		driveSolLeft.set(isHigh ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
		driveSolRight.set(!isHigh ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
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
		SmartDashboard.putBoolean("help me", isVBus);
		
		motors[0].set(ControlMode.PercentOutput,left);
		motors[2].set(ControlMode.PercentOutput,right);
	}
	public void driveForwardRotateTeleop(double fwd, double rot) {
		rot = -rot;
		if (rot > 0) {
			rot = Math.pow(rot, 1);
		} else {
			rot = -Math.pow(Math.abs(rot), 1);
		}
		driveFwdRotate(fwd, rot, true);
	}
	public void rawDrive(double left, double right) {
		motors[0].set(ControlMode.PercentOutput, -left);
		motors[1].set(ControlMode.Follower, RobotMap.Drivetrain.MOTOR_PORTS[0]);
		motors[2].set(ControlMode.PercentOutput, right);
		motors[3].set(ControlMode.Follower, RobotMap.Drivetrain.MOTOR_PORTS[2]);
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
		double encPos = -(double) motors[2].getSensorCollection().getQuadraturePosition();
		if (!isHighGear) {
			return ((encPos)/4096.0) *(14.0/60.0) * (5.0 * Math.PI) / 2;
		} else {
			return ((encPos)/4096.0) *(24.0/50) * (5.0 * Math.PI) / 2;
		}
	}
	public double getAvgTalonDistance() {
		return (getTalonDistanceLeft() + getTalonDistanceRight())/ 2.0;
	}
	
	public void resetEncoders() {
		motors[0].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
		motors[2].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
//		leftEncoder.reset();
	}
	public double getTranslationDistance(double angle) {
		return convert(32 * Math.PI * (angle / 360));
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
	
	
	public int convert(double d){
		int i = (int)((scalingFactor*d*50*4096)/(Math.PI*wheelSize*24));
		return i;
	}
	public void changeScale(int x){
		if(scalingFactor+x!=0)
		scalingFactor+=x;
	}
	public void changeWheel(double x){
		if(wheelSize+x!=0)
		wheelSize+=x;
	}
	
}

