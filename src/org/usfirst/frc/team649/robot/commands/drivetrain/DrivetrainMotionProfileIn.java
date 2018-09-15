package org.usfirst.frc.team649.robot.commands.drivetrain;

import org.usfirst.frc.team649.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainMotionProfileIn extends Command {

	double setpoint;
	boolean isFinished;
	double converted;
    public DrivetrainMotionProfileIn(double setpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.setpoint = setpoint;
    	isFinished = false;

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	for (int i = 0; i < 4; i++) {
			Robot.drive.motors[i].setNeutralMode(NeutralMode.Brake);
			Robot.drive.motors[i].configMotionAcceleration(9000, Robot.timeoutMs);
			Robot.drive.motors[i].configMotionCruiseVelocity(18000, Robot.timeoutMs);
			Robot.drive.motors[i].config_kP(0, 1.5, Robot.timeoutMs);
			Robot.drive.motors[i].config_kI(0, 0, Robot.timeoutMs);
			Robot.drive.motors[i].config_kD(0, 0.9, Robot.timeoutMs);
		}
    	
    	isFinished = false;
    	
    	Robot.drive.changeBrakeCoast(true);
    	Robot.drive.shift(true);
    	
    	Robot.isDrivePIDRunning = true;
    	
    	converted = Robot.drive.convert(setpoint);
    	SmartDashboard.putNumber("Converted", converted);	
    	
    	Robot.drive.resetEncoders();
    	
		SmartDashboard.putNumber("Setpoint", setpoint);
		SmartDashboard.putString("Current Command", getName() + setpoint);
    	SmartDashboard.putBoolean("Is TalonDistance Finished?", false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.motors[0].set(ControlMode.MotionMagic,(int) converted );
		Robot.drive.motors[2].set(ControlMode.MotionMagic, (int)converted);

		
    	SmartDashboard.putNumber("Left Talon Distance", Robot.drive.getTalonDistanceLeft());
    	
    	if (Math.abs(converted - Robot.drive.motors[0].getSelectedSensorPosition(0)) < 100) {
			isFinished = true;
		}
//    	if(Robot.auto.get() > 14.8){
//    		isFinished = true;
//    	}
    } 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	SmartDashboard.putBoolean("Is TalonDistance Finished?", true);
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.motors[0].set(ControlMode.PercentOutput, 0);
    	Robot.drive.motors[2].set(ControlMode.PercentOutput, 0);
    	Robot.isDrivePIDRunning = false;
//    	Robot.drive.changeBrakeCoast(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();

    }
}
