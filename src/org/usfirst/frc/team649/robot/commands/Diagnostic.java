package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
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
		public static final int armMotor = 5;
	}
	boolean prevStateIntake;
	boolean prevStateBrake;
	boolean prevStateShift;
	boolean intakeMotorRight;
	int state;
	boolean isFinished;
	Timer time1;
	Timer time2;
	Timer time3;
	Timer time4;
	
	
    public Diagnostic() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	isFinished = false;
    	time1 = new Timer();
    	time2 = new Timer();
    	time3 = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	prevStateIntake = solState(Robot.intake.intakeSol.get());
    	prevStateBrake = solState(Robot.arm.armBrake.get());
    	prevStateShift = solState(Robot.drive.driveSolRight.get());
    	state = -1;
    	
    	
    	time1.start();
    	time2.start();
    	time3.start();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.oi.operatorJoystick.getRawButton(2)&& time1.get() > 0.2) {
    		Robot.intake.setIntakePiston30(!prevStateIntake);
    		prevStateIntake = !prevStateIntake;
    		SmartDashboard.putBoolean("Intake State", prevStateIntake);
    		time1.reset();
    		time1.start();
    	}
    	if (Robot.oi.operatorJoystick.getPOV() == 0 && time1.get() > 0.2) {
    		Robot.intake.setIntakePiston60(!prevStateIntake);
    		prevStateIntake = !prevStateIntake;
    		SmartDashboard.putBoolean("Intake State", prevStateIntake);
    		time1.reset();
    		time1.start();
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(3) && time2.get() > 0.2) {
    		Robot.arm.setArmBrake(!prevStateBrake);
    		prevStateBrake = !prevStateBrake;
    		SmartDashboard.putBoolean("Brake State", prevStateIntake);
    		time2.reset();
    		time2.start();
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(4) && time3.get() > 0.2) {
    		Robot.drive.shift(!prevStateShift);
    		prevStateShift = !prevStateShift;
    		SmartDashboard.putBoolean("Shift State", prevStateIntake);
    		time3.reset();
    		time3.start();
    		
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(5)) {
    		state = Constants.intakeMotorLeft;
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(6)) {
    		state = Constants.intakeMotorRight;
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(7)) {
    		state = Constants.liftMotor;
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(8)) {
    		state = Constants.drivetrainMotorLeft;
    	}
    	if (Robot.oi.operatorJoystick.getRawButton(9)) {
    		state = Constants.drivetrainMotorRight;
    	}
    	if(Robot.oi.operatorJoystick.getRawButton(10)) {
    		state = Constants.armMotor;
    	}
//    	if(Robot.oi.operator.PIDTunePhase()) {
//    		state = -1;
//    	}
//    	if (Robot.oi.operator.getButton13()) {
//    		isFinished = true;
//    	}
    	
    	SmartDashboard.putBoolean("Diag?", true);
    	switch(state) {
    	case Constants.intakeMotorLeft:
    		Robot.intake.setIntakeMotors(0, Robot.oi.operator.getOperatorY());
    		Robot.drive.rawDrive(0, 0);
    		Robot.lift.setLift(0);
    		Robot.arm.setArm(0);
    		SmartDashboard.putString("Currently Running Motor: ", "Intake Left");
    		break;
    	case Constants.intakeMotorRight:
    		Robot.intake.setIntakeMotors(Robot.oi.operator.getOperatorY(), 0);
    		Robot.drive.rawDrive(0, 0);
    		Robot.lift.setLift(0);
    		Robot.arm.setArm(0);
    		SmartDashboard.putString("Currently Running Motor: ", "Intake Right");
    		break;
    	case Constants.liftMotor:
    		Robot.lift.setLift(Robot.oi.operator.getOperatorY());
    		SmartDashboard.putBoolean("Direction Correct?", 
    				checkSign(Robot.oi.operator.getOperatorY(), Robot.lift.getRawLiftVel()));
    		Robot.intake.setIntakeMotors(0, 0);
    		Robot.drive.rawDrive(0, 0);
    		Robot.arm.setArm(0);
    		SmartDashboard.putString("Currently Running Motor: ", "Lift");
    		break;
    	case Constants.drivetrainMotorLeft:
    		Robot.drive.rawDrive(Robot.oi.operator.getOperatorY(), 0);
    		SmartDashboard.putBoolean("Direction Correct?", 
    				checkSign(Robot.oi.operator.getOperatorY(), Robot.drive.motors[0].getSelectedSensorVelocity(0)));
    		Robot.intake.setIntakeMotors(0, 0);
    		Robot.lift.setLift(0);
    		Robot.arm.setArm(0);
    		SmartDashboard.putString("Currently Running Motor: ", "Drive Left");
    		break;
    	case Constants.drivetrainMotorRight:
    		Robot.drive.rawDrive(0, Robot.oi.operator.getOperatorY());
    		SmartDashboard.putBoolean("Direction Correct?", 
    				checkSign(-Robot.oi.operator.getOperatorY(), Robot.drive.motors[2].getSelectedSensorVelocity(0)));
    		Robot.intake.setIntakeMotors(0, 0);
    		Robot.lift.setLift(0);
    		Robot.arm.setArm(0);
    		SmartDashboard.putString("Currently Running Motor: ", "Drive Right");
    		break;
    	case Constants.armMotor:
    		Robot.arm.setArm(Robot.oi.operator.getOperatorY());
    		SmartDashboard.putBoolean("Direction Correct?", 
    				checkSign(Robot.oi.operator.getOperatorY(), Robot.arm.bottomMotor.getSelectedSensorVelocity(0)));
    		Robot.intake.setIntakeMotors(0, 0);
    		Robot.drive.rawDrive(0, 0);
    		Robot.lift.setLift(0);
    		SmartDashboard.putString("Currently Running Motor: ", "Arm");
    		break;
    	case -1:
    		SmartDashboard.putString("Currently Running Motor: ", "None");
    		break;
    	}
    	SmartDashboard();
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
    	SmartDashboard.putBoolean("Arm back", Robot.arm.getArmHalZeroBack());
    	SmartDashboard.putBoolean("Arm front", Robot.arm.getArmHalZeroFront());
    	// drivetrain
    	SmartDashboard.putNumber("lidar", Robot.lidar.getSample());
    	SmartDashboard.putNumber("DT Left Side Encoder", Robot.drive.motors[0].getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("DT Right Side Encoder", Robot.drive.motors[2].getSelectedSensorPosition(0));
    	
    }
    public boolean checkSign(double a, double b) {
    	if((a > 0 && b > 0) || (a < 0 && b < 0)) {
    		return true;
    	}
    	return false;
    }
}
