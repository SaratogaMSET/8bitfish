package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.MotionProfileDrive.PeriodicRunnable;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

/**
 *
 */
public class MotionProfileDriveInverted extends Command {

	boolean back;
    Notifier run;
    boolean isFinished;
    Timer timeout;

    
    public MotionProfileDriveInverted(boolean isBack) {
    	back = isBack;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.isMPRunning = true;
    	SmartDashboard.putBoolean("isMPFin", false);
    	run = new Notifier(new Runny());
    	timeout = new Timer();
    	timeout.start();
    	isFinished = false;
//    	Robot.drive.motors[0].setSensorPhase(true);
//    	Robot.drive.motors[2].setSensorPhase(true);
        run.startPeriodic(0.05);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.isMPRunning = true;
    	SmartDashboard.putBoolean("isMPFin", false);
    	timeout = new Timer();
    	timeout.start();
    	isFinished = false;
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
    class Runny implements Runnable {
        
        @Override
        public void run() {
        	double l = Robot.left.calculate(-Robot.drive.motors[0].getSelectedSensorPosition(0));
    		double r = Robot.right.calculate(-Robot.drive.motors[2].getSelectedSensorPosition(0));

    		double gyro_heading = Robot.gyro.getGyroAngle();
    		double desired_heading = Pathfinder.r2d(Robot.left.getHeading());  
    		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading + gyro_heading);
    		//1.1 for middle left
    		double turn;
    		if(Robot.shouldSwitchTurnRatio){
    			turn = 2* (-1.0/80.0) * angleDifference;

    		}else{
    			turn = 2 * (-1.0/80.0) * angleDifference;
    		}
    		if(back){
    			Robot.drive.motors[0].set(ControlMode.PercentOutput, -l);
    			Robot.drive.motors[2].set(ControlMode.PercentOutput, -r);
    		}else{
    			//NEVER GO ABOVE 1
    			Robot.drive.motors[0].set(ControlMode.PercentOutput, -(l-turn));
    			Robot.drive.motors[2].set(ControlMode.PercentOutput, -(r+turn));
    		}
    		SmartDashboard.putNumber("turn", turn);
    		if((Robot.left.isFinished()&&Robot.right.isFinished())||timeout.get()>14.95){
    			isFinished = true;
    			SmartDashboard.putBoolean("fin", true);
    		}
        }
    }
}
