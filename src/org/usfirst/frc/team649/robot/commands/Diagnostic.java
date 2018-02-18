package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Diagnostic extends Command {
	
	public static class Constants {
		public static final int intakeMotorLeft= 0;
		public static final int intakeMotorRight = 4;
		public static final int  liftMotor = 1;
		public static final int drivetrainMotorLeft = 2;
		public static final int drivetrainMotorRight = 3;
	}
	boolean prevStateIntake;
	boolean prevStateBrake;
	boolean prevStateShift;
	boolean intakeMotorRight;
	int state;
	boolean isFinished;
	
    public Diagnostic() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	isFinished = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	prevStateIntake = solState(Robot.intake.intakeSol.get());
    	prevStateBrake = solState(Robot.arm.armBrake.get());
    	prevStateShift = solState(Robot.drive.driveSolRight.get());
    	state = -1;

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.oi.operator.getButton2()) {
    		Robot.intake.setIntakePiston(!prevStateIntake);
    		prevStateIntake = !prevStateIntake;
    	}
    	if (Robot.oi.operator.getButton3()) {
    		Robot.arm.setArmBrake(!prevStateBrake);
    		prevStateBrake = !prevStateBrake;
    	}
    	if (Robot.oi.operator.getButton4()) {
    		Robot.drive.shift(!prevStateShift);
    		prevStateShift = !prevStateShift;
    	}
    	if (Robot.oi.operator.getButton5()) {
    		state = Constants.intakeMotorLeft;
    	}
    	if (Robot.oi.operator.getButton6()) {
    		state = Constants.intakeMotorRight;
    	}
    	if (Robot.oi.operator.getButton7()) {
    		state = Constants.liftMotor;
    	}
    	if (Robot.oi.operator.getButton8()) {
    		state = Constants.drivetrainMotorLeft;
    	}
    	if (Robot.oi.operator.getButton9()) {
    		state = Constants.drivetrainMotorRight;
    	}
    	if(Robot.oi.operator.PIDTunePhase()) {
    		state = -1;
    	}
    	if (Robot.oi.operator.getButton13()) {
    		isFinished = true;
    	}
    	
    	switch(state) {
    	case Constants.intakeMotorLeft:
    		Robot.intake.setIntakeMotors(0, Robot.oi.operator.getOperatorY());
    		SmartDashboard.putString("Currently Running Motor: ", "Intake Left");
    		break;
    	case Constants.intakeMotorRight:
    		Robot.intake.setIntakeMotors(Robot.oi.operator.getOperatorY(), 0);
    		SmartDashboard.putString("Currently Running Motor: ", "Intake Right");
    		break;
    	case Constants.liftMotor:
    		Robot.lift.setLift(Robot.oi.operator.getOperatorY());
    		SmartDashboard.putString("Currently Running Motor: ", "Lift");
    		break;
    	case Constants.drivetrainMotorLeft:
    		Robot.drive.rawDrive(Robot.oi.operator.getOperatorY(), 0);
    		SmartDashboard.putString("Currently Running Motor: ", "Drive Left");
    		break;
    	case Constants.drivetrainMotorRight:
    		Robot.drive.rawDrive(0, Robot.oi.operator.getOperatorY());
    		SmartDashboard.putString("Currently Running Motor: ", "Drive Right");
    		break;
    	case -1:
    		SmartDashboard.putString("Currently Running Motor: ", "None");
    		break;
    	}
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
    public boolean solState(DoubleSolenoid.Value val) {
    	if (val == DoubleSolenoid.Value.kForward) {
    		return true;
    	} else {
    		return false;
    	}
    }
    public void SmartDashboard() {
    	// arm
    	SmartDashboard.putBoolean("Arm IR", Robot.arm.getInfraredSensor());
    	SmartDashboard.putNumber("Arm Raw Position", Robot.arm.bottomMotor.getSelectedSensorPosition(0));
    	
    	// lift
    	SmartDashboard.putBoolean("Bottom Carriage Hal", Robot.lift.isCarriageAtBottom());
    	SmartDashboard.putBoolean("Top Carriage Hal", Robot.lift.isCarriageAtTop());
    	SmartDashboard.putBoolean("Bottom Sec Stage Hal", Robot.lift.isSecondStageAtBottom());
    	SmartDashboard.putBoolean("Top Sec Stage Hal", Robot.lift.isSecondStageAtTop());
    	SmartDashboard.putNumber("Lift Raw", Robot.lift.mainLiftMotor.getSelectedSensorPosition(0));
    	
    	// drivetrain
    	SmartDashboard.putNumber("DT Right Side Encoder", Robot.drive.getTalonDistanceRight());
    	SmartDashboard.putNumber("DT Left Side Encoder", Robot.drive.getTalonDistanceLeft());
    	
    }
}
