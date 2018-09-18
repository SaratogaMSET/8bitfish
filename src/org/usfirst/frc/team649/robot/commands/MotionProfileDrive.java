package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

/**
 *
 */
public class MotionProfileDrive extends Command {
	boolean back;
    Notifier periodicRunnable;
    boolean isFinished;
    Timer timeout;

    public MotionProfileDrive(boolean isBack) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	back = isBack;

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.isMPRunning = true;
    	SmartDashboard.putBoolean("isMPFin", false);
    	periodicRunnable = new Notifier(new PeriodicRunnable());
    	timeout = new Timer();
    	timeout.start();
    	isFinished = false;
        periodicRunnable.startPeriodic(0.05);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double l = Robot.left.calculate(Robot.drive.motors[0].getSelectedSensorPosition(0));
		double r = Robot.right.calculate(Robot.drive.motors[2].getSelectedSensorPosition(0));

		double gyro_heading = -Robot.gyro.getGyroAngle();
		double desired_heading = Pathfinder.r2d(Robot.left.getHeading());  
		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		//1.1 for middle left
		double turn;
		if(Robot.shouldSwitchTurnRatio){
			turn = 1.1 * (-1.0/80.0) * angleDifference;

		}else{
			turn = 0.8 * (-1.0/80.0) * angleDifference;
		}
		if(back){
			Robot.drive.motors[0].set(ControlMode.PercentOutput, l);
			Robot.drive.motors[2].set(ControlMode.PercentOutput, r);
		}else{
			Robot.drive.motors[0].set(ControlMode.PercentOutput, l+turn);
			Robot.drive.motors[2].set(ControlMode.PercentOutput, r-turn);
		}
		SmartDashboard.putNumber("turn", turn);
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SmartDashboard.putBoolean("isMPFin", true);
    	Robot.isMPRunning = false;


    	Robot.drive.rawDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();

    }
    class PeriodicRunnable implements Runnable {
       
        @Override
        public void run() {
        	double l = Robot.left.calculate(Robot.drive.motors[0].getSelectedSensorPosition(0));
    		double r = Robot.right.calculate(Robot.drive.motors[2].getSelectedSensorPosition(0));

//    		double gyro_heading = -Robot.gyro.getGyroAngle();
//    		double desired_heading = Pathfinder.r2d(Robot.left.getHeading());  
//    		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    		double gyro_heading = Robot.gyro.getGyroAngle();
    		double desired_heading = Pathfinder.r2d(Robot.left.getHeading());  
    		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading + gyro_heading);
    		//1.1 for middle left
    		double turn;
    		if(Robot.shouldSwitchTurnRatio){
    			turn =  1.5 * (-1.0/80.0) * angleDifference;

    		}else{
    			turn = 3.5 * (-1.0/80.0) * angleDifference;
    		}
    		if(back){
    			Robot.drive.motors[0].set(ControlMode.PercentOutput, l);
    			Robot.drive.motors[2].set(ControlMode.PercentOutput, r);
    		}else{
    			Robot.drive.motors[0].set(ControlMode.PercentOutput, l+turn);
    			Robot.drive.motors[2].set(ControlMode.PercentOutput, r-turn);
    		}
    		SmartDashboard.putNumber("turn", turn);
    		if((Robot.left.isFinished()&&Robot.right.isFinished()&&Math.abs(angleDifference) < 4)||timeout.get()>14.95){
    			isFinished = true;
    		}
    		
//            // Calculate the current motor outputs based on the trajectory values + encoder positions
//            double l = left.calculate(Robot.driveTrain.getLeftEncoderValue());
//            double r = right.calculate(Robot.driveTrain.getRightEncoderValue());
//           
//            // Adjust a turn value based on the gyro's heading + the trajectory's heading. Note that we only use the left's heading, but left/right would be the same since they're following the same path, but separated by wheelbase distance.
//            double angleDifference = 0;
//            double turn = 0;
//
//            try {
//                angleDifference = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()) + Robot.driveTrain.spartanGyro.getAngle());
//                turn = 2 * (-1.0 / 80.0) * angleDifference;
//            } catch (Exception e) {
//                angleDifference = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));
//                turn = 2 * (-1.0 / 80.0) * angleDifference;
//            }
//
//            // Set the output to the motors
//            Robot.driveTrain.setDirectDriveOutput(l + turn, r - turn);
//
//            // Continue sending output values until the path has been completely followed.
//            if (left.isFinished() && right.isFinished() && (Math.abs(angleDifference) < 4)) {
//                end = true;
//            }
        }
    }
}
