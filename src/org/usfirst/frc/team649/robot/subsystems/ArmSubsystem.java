package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ArmSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public static class ArmConstants{
		public static final double ARM_POWER = 0.4;
	}
	DigitalInput infraredSensor;
	TalonSRX bottomMotor,topMotor;
	public ArmSubsystem() {
		bottomMotor = new TalonSRX(RobotMap.Arm.BOTTOM_ARM_MOTOR);
		topMotor = new TalonSRX(RobotMap.Arm.TOP_ARM_MOTOR);
		infraredSensor = new DigitalInput(RobotMap.Arm.INFRARED_SENSOR);
	}
	public void setArm(double power){
		bottomMotor.set(ControlMode.PercentOutput, -power);
		topMotor.set(ControlMode.PercentOutput, power);
	}
	public boolean getInfraredSensor() {
		return infraredSensor.get();
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

