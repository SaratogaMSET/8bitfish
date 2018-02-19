package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**f
 *
 */
public class GyroStraightPID extends Command {
	
	PIDController GyroController;
	PIDController DriveController;
	double angle;
	double distance;
	boolean isFinished;
	boolean atTurningPoint;
	double prevDistLeft;
	double prevDistRight;
	double prevVelLeft;
	double prevVelRight;
	Timer doneTime;
	double turnPoint;
	double accel = 11000;
	double gyroFinal;
	
    public GyroStraightPID(double distance, double angle, double turnPoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.distance = distance;
    	doneTime = new Timer();
    	GyroController = Robot.gyro.getPIDController();
    	DriveController = Robot.drive.getPIDController();
    	gyroFinal = 0;
    	this.angle = angle;
    	this.turnPoint = turnPoint;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("finished gyro", false);
//    	Robot.gyro.setDrivingStraight(true);
    	Robot.drivePIDRunning = true;
    	GyroController.enable();
    	prevDistLeft = 0;
    	prevDistRight = 0;
    	prevVelLeft = Robot.drive.motors[0].getSelectedSensorVelocity(0);
    	prevVelRight = Robot.drive.motors[2].getSelectedSensorVelocity(0);
    	isFinished = false;
//    	DriveController.enable();
    	Robot.gyro.resetGyro();
//    	GyroController.setSetpoint(0);
    	atTurningPoint = false;
//    	DriveController.setSetpoint(distance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double left = 0;
    	double right = 0;
//    	double drivePower = Robot.drive.getDrivePIDOutput();
    	if ((Math.abs(Robot.drive.motors[0].getSelectedSensorPosition(0)) + Math.abs(Robot.drive.motors[2].getSelectedSensorPosition(0)))/2 > turnPoint) {
    		atTurningPoint = true;
//    		Robot.drive.motors[0].configMotionCruiseVelocity(10000, 20);
//    		Robot.drive.motors[2].configMotionCruiseVelocity(10000, 20);
    	}
    	SmartDashboard.putBoolean("turning point", atTurningPoint);
    	double gyroPower;
    	
    	double mult;
    	if(atTurningPoint) {
    		gyroPower = angle - Robot.gyro.getGyroAngle();
    		mult = 1200;
    	} else {
    		gyroPower = Robot.gyro.getGyroAngle();
    		mult = 400;
    	}
    	if ( gyroPower > 0) {
    		if(prevVelRight < Math.abs(Robot.drive.motors[0].getSelectedSensorVelocity(0))){
    			left = accel - gyroPower*mult;
    			right = accel;
    		}else{
    			right = accel- gyroPower * mult;
    			left = accel ;
    		}

		} else if (gyroPower < 0) {
			if(prevVelLeft < Math.abs(Robot.drive.motors[2].getSelectedSensorVelocity(0))){
				right = accel- gyroPower * mult;
				left = accel ;
			}else{
				left = accel- gyroPower*mult;
    			right = accel ;
			}
			
		} else {
			right = accel;
			left = accel;
		}
    	if(left < 0){
    		left = 0;
    	}
    	if(right < 0){
    		right = 0;
    	}
    	SmartDashboard.putNumber("Righ accel",right );
    	SmartDashboard.putNumber("Left accel", left);
    	prevVelLeft = Robot.drive.motors[0].getSelectedSensorVelocity(0);
    	prevVelRight = Robot.drive.motors[2].getSelectedSensorVelocity(0);
    	
//    	double max = Math.max(right, Math.max(left, 1));
    	
//    	right /= max;
//    	left /= max;
//		Robot.drive.rawDrive(left, right);
    	Robot.drive.motors[0].configMotionAcceleration((int) left, 20);
        Robot.drive.motors[2].configMotionAcceleration((int) right, 20);
    	
		Robot.drive.motors[0].set(ControlMode.MotionMagic, distance);
		Robot.drive.motors[2].set(ControlMode.MotionMagic, -distance);
		SmartDashboard.putNumber("gyro pid value", gyroPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//    	if ((Math.abs((distance-Math.abs(Robot.drive.motors[0].getSelectedSensorPosition(0))))+Math.abs((distance-Math.abs(Robot.drive.motors[2].getSelectedSensorPosition(0))))) < 50) {
//    		return true;
//    	}
    	if(doneTime.get() > 0.25){
    		gyroFinal = Robot.gyro.getGyroAngle();
    		SmartDashboard.putNumber("Gyro Final", gyroFinal);
    		return true;
    	}else if(doneTime.get()>0 && (Math.abs(prevDistLeft-Math.abs(Robot.drive.motors[0].getSelectedSensorPosition(0))) < 15) || (Math.abs(prevDistRight-Math.abs(Robot.drive.motors[2].getSelectedSensorPosition(0))) < 15)){
    		doneTime.stop();
    		doneTime.reset();
    	}
    	else if(doneTime.get() == 0 && (Math.abs(distance)-Math.abs(Robot.drive.motors[0].getSelectedSensorPosition(0)) + (Math.abs(distance)-Math.abs(Robot.drive.motors[2].getSelectedSensorPosition(0))) < 100)){
    		doneTime.start();
    	}
    	prevDistLeft = Robot.drive.motors[0].getSelectedSensorPosition(0);
    	prevDistRight = Robot.drive.motors[2].getSelectedSensorPosition(0);

    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivePIDRunning = false;
//    	GyroController.disable();
//    	DriveController.disable();
    	SmartDashboard.putNumber("end left", Robot.drive.motors[0].getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("end right", Robot.drive.motors[2].getSelectedSensorPosition(0));
    	SmartDashboard.putBoolean("finished gyro", true);
    	Robot.drive.motors[0].set(ControlMode.PercentOutput, 0);
		Robot.drive.motors[2].set(ControlMode.PercentOutput, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
