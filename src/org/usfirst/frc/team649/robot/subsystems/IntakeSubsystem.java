package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeSubsystem extends Subsystem {

    public TalonSRX leftIntake, rightIntake;
    public DoubleSolenoid intakeSol; //60
    public DoubleSolenoid intakeSol2; //30

    public boolean isIntakeRunning;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    	//when trying to intake closed, close both
  		//while trying to clamp cube run only 60
  		//open run just 30
    public IntakeSubsystem(){
    	intakeSol = new DoubleSolenoid(RobotMap.Intake.INTAKE_SOL[0],RobotMap.Intake.INTAKE_SOL[1],RobotMap.Intake.INTAKE_SOL[2]);
    	intakeSol2 = new DoubleSolenoid(RobotMap.Intake.SECOND_SOL[0],RobotMap.Intake.SECOND_SOL[1],RobotMap.Intake.SECOND_SOL[2]);
    	
    	leftIntake = new TalonSRX(RobotMap.Intake.LEFT_INTAKE_MOTOR);
    	rightIntake = new TalonSRX(RobotMap.Intake.RIGHT_INTAKE_MOTOR);
    	
    	leftIntake.configNominalOutputForward(0, Robot.timeoutMs);
    	leftIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    	leftIntake.configPeakOutputForward(1, Robot.timeoutMs);
    	leftIntake.configPeakOutputReverse(-1, Robot.timeoutMs);
    	
    	rightIntake.configNominalOutputForward(0, Robot.timeoutMs);
    	rightIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    	rightIntake.configPeakOutputForward(1, Robot.timeoutMs);
    	rightIntake.configPeakOutputReverse(-1, Robot.timeoutMs);
    	
    	isIntakeRunning = false;
    }
    public void setIntakeMotors(double right, double left){
    	leftIntake.set(ControlMode.PercentOutput, left);
    	rightIntake.set(ControlMode.PercentOutput, right);
    	if (Math.abs(left) > 0 || Math.abs(right) > 0) {
    		isIntakeRunning = true;
    	}
    }
    public void setIntakePiston60(boolean isOut){
		intakeSol.set(!isOut ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
    public void setIntakePiston30(boolean isOut){
  		intakeSol2.set(!isOut ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public boolean isRunning()
    {
    	return isIntakeRunning;
    }
    
}

