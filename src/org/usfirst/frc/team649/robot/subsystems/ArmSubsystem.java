package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
		public static final int RAW_ABS_TOL = 15;
	}
	DigitalInput infraredSensor;
	public TalonSRX bottomMotor;
	public TalonSRX topMotor;
	public double lastVal;
	public Timer time;
	public DoubleSolenoid armBrake;
	public ArmSubsystem() {
		bottomMotor = new TalonSRX(RobotMap.Arm.BOTTOM_ARM_MOTOR);
		bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Robot.timeoutMs);
		topMotor = new TalonSRX(RobotMap.Arm.TOP_ARM_MOTOR);
		infraredSensor = new DigitalInput(RobotMap.Arm.INFRARED_SENSOR);
		bottomMotor.configNominalOutputForward(0, Robot.timeoutMs);
		bottomMotor.configNominalOutputReverse(0, Robot.timeoutMs);
		bottomMotor.configPeakOutputForward(1.0, Robot.timeoutMs);
		bottomMotor.configPeakOutputReverse(-1.0, Robot.timeoutMs);
		bottomMotor.setSensorPhase(false);
		bottomMotor.setInverted(false);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Robot.timeoutMs);
		lastVal = 0;
		time = new Timer();
		time.start();
		bottomMotor.configMotionAcceleration(850, Robot.timeoutMs);
		bottomMotor.configMotionCruiseVelocity(700, Robot.timeoutMs);
		bottomMotor.config_kP(0, 5, Robot.timeoutMs);
		bottomMotor.config_kI(0, 0.004, Robot.timeoutMs);
		bottomMotor.config_kD(0, 0.01, Robot.timeoutMs);
		bottomMotor.config_kF(0, 1.25, Robot.timeoutMs);
		bottomMotor.selectProfileSlot(0, 0);
		topMotor.setInverted(true);
		topMotor.set(ControlMode.Follower, 17);
		armBrake = new DoubleSolenoid(RobotMap.Arm.ARM_BRAKE[0],RobotMap.Arm.ARM_BRAKE[1],RobotMap.Arm.ARM_BRAKE[2]);
		

	}
	public void setArm(double power){
		bottomMotor.set(ControlMode.PercentOutput, -power);
		topMotor.set(ControlMode.Follower, 17);
	}
	public boolean getInfraredSensor() {
		return infraredSensor.get();
	}
	public void setArmBrake(boolean isEngaged){
		armBrake.set(!isEngaged ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
	}
	public int getArmRaw() {
		return bottomMotor.getSensorCollection().getPulseWidthPosition();
	}
	
	public double getArmPosition() {
		return getArmRaw()/4096.0 /2.0;
	}
	
	public double getArmAngle() {
		return getArmRaw()/4050.0 * 360.0;
	}
	
	public double getVel() {
		lastVal = bottomMotor.getSensorCollection().getQuadratureVelocity();
		return lastVal;
	}
	
	public double getAccel() {
		return (bottomMotor.getSensorCollection().getQuadratureVelocity() - lastVal);
	}
	
	public void resetEncoder() {
		bottomMotor.getSensorCollection().setQuadraturePosition(0, 20);
	}
	
	public double getTime() {
		return time.get();
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

