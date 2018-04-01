package org.usfirst.frc.team649.test;

import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.CommandGroups.DeployWithWheelsAndIntake;
import org.usfirst.frc.team649.robot.CommandGroups.IntakeWithWheelsAndClose;
import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
import org.usfirst.frc.team649.robot.commands.SetIntakePistons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoTestCommand extends Command {
	
	Timer time;
	boolean isFinished;
    public AutoTestCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	time = new Timer();
    	isFinished = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	time.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	if(Robot.oi.operator.getButton2()) {
//			if(Robot.program == 1) {
//				Robot.program = 2;
//			} else if (Robot.program == 2) {
//				Robot.program = 1;
//			}
//		}
//    	if(Robot.oi.buttonBoard.getRawButton(4) && time.get()>0.2) {
//    		Robot.autoTest.incrementTuningVarValue(true);
//    		time.reset();
//    	}
//    	if(Robot.oi.buttonBoard.getRawButton(6) && time.get()>0.2) {
//    		Robot.autoTest.incrementTuningVarValue(false);
//    		time.reset();
//    	}
//    	if(Robot.oi.buttonBoard.getRawButton(5) && time.get()>0.2) {
//    		Robot.autoTest.changeTuningIncrement(true);
//    		time.reset();
//    	}
//    	if(Robot.oi.buttonBoard.getRawButton(7) && time.get()>0.2) {
//    		Robot.autoTest.changeTuningIncrement(false);
//    		time.reset();
//    	}
    	if(Robot.oi.buttonBoard.getRawButton(2) && time.get()>0.2) {
    		Robot.autoTest.changeCurrentlyTuningProgram(true);
    		time.reset();
    	}
    	if(Robot.oi.buttonBoard.getRawButton(3) && time.get()>0.2) {
    		Robot.autoTest.changeCurrentlyTuningProgram(true);
    		time.reset();
    	}
//    	if(Robot.oi.buttonBoard.getRawButton(8) && time.get()>0.2) {
//    		Robot.autoTest.changeCurrentlyTuningSegment(true);
//    		time.reset();
//    	}
//    	if(Robot.oi.buttonBoard.getRawButton(9) && time.get()>0.2) {
//    		Robot.autoTest.changeCurrentlyTuningSegment(false);
//    		time.reset();
//    	}
    	if(Robot.oi.operatorJoystick.getRawButton(11) && time.get()>0.2) {
    		isFinished = true;
    		Robot.autoTest.startProgram();
    		time.reset();
    	}
    	if(Robot.oi.operator.deployOnlyWheels()){
			new RunIntakeWheels(-1).start();
		}else if(Robot.oi.operator.deployWithWheelsAndOpen()){
			new DeployWithWheelsAndIntake().start();;
		}else if(Robot.oi.operator.openIntakeToggle()){
			new SetIntakePistons(true, true).start();
		}else if(Robot.oi.operator.runIntakeWithWheelsClosed()){
			new IntakeWithWheelsAndClose().start();
		}else if(Robot.oi.operator.closeIntake()){
			new SetIntakePistons(false, false).start();
		}else{
			Robot.intake.setIntakeMotors(0, 0);
		}
    	Robot.drive.driveFwdRotate(Robot.oi.driver.getForward(), Robot.oi.driver.getRotation(), true);
    	Robot.autoTest.SmartDashboard();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
