package org.usfirst.frc.team649.robot.subsystems;

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
		public static final double ARM_POWER = 0.4;
		public static final int RAW_ABS_TOL = 10;
	}
	DigitalInput infraredSensor;
	public TalonSRX bottomMotor;
	public TalonSRX topMotor;
	public double lastVal;
	public Timer time;
	public DoubleSolenoid armBrake;
	public ArmSubsystem() {
		bottomMotor = new TalonSRX(RobotMap.Arm.BOTTOM_ARM_MOTOR);
		bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 30);
		topMotor = new TalonSRX(RobotMap.Arm.TOP_ARM_MOTOR);
		infraredSensor = new DigitalInput(RobotMap.Arm.INFRARED_SENSOR);
		bottomMotor.configNominalOutputForward(0, 30);
		bottomMotor.configNominalOutputReverse(0, 30);
		bottomMotor.configPeakOutputForward(0.5, 30);
		bottomMotor.configPeakOutputReverse(-0.5, 30);
		bottomMotor.setSensorPhase(false);
		bottomMotor.setInverted(false);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
		lastVal = 0;
		time = new Timer();
		time.start();
		bottomMotor.configMotionAcceleration(250, 20);
		bottomMotor.configMotionCruiseVelocity(610, 20);
		bottomMotor.config_kP(0, 2, 20);
		bottomMotor.config_kI(0, 0.0, 20);
		bottomMotor.config_kD(0, 0.01, 20);
		bottomMotor.config_kF(0, 0.3599, 20);
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
	public double getArmRaw() {
		return bottomMotor.getSensorCollection().getQuadraturePosition();
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

