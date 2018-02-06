package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LiftSubsystem extends PIDSubsystem {

    public static class LiftPIDConstants{
    	public static double k_P = 0;
    	public static double k_I = 0;
    	public static double k_D = 0;
    	public static double SECOND_STAGE_TRANSLATION_CONSTANT = 0;
    	public static double CARRIAGE_STAGE_TRANSLATION_CONSTANT = 0;
    	public static double MAX_SECOND_STAGE_HEIGHT = 0; //inches
    	public static double MAX_TOTAL_HEIGHT = 0;
    	public static double ABS_TOL = 0;
    }
    public static class LiftPowerValues{
    	public static double LIFT_POWER = 0.5;
    }
    public static class LiftStateConstants{
    	public static int LOWEST_STATE = 1;
    	public static int CARRIAGE_LOW_SECOND_MID = 2;
    	public static int CARRIAGE_LOW_SECOND_HIGH = 3;
    	public static int CARRIAGE_MID_SECOND_HIGH = 4;
    	public static int CARRIAGE_HIGH_SECOND_HIGH = 5;
    	public static int CARRIAGE_HIGH_SECOND_MID = 6;
    	public static int CARRIAGE_HIGH_SECOND_LOW = 7;
    	public static int CARRIAGE_MID_SECOND_LOW = 8;
    }
    public TalonSRX mainLiftMotor,followerLiftMotor;
    public DigitalInput botSecondStageHal, topSecondStageHal, botCarriageHal, topCarriageHal;
    public LiftSubsystem(){
    	super(LiftPIDConstants.k_P, LiftPIDConstants.k_I, LiftPIDConstants.k_D);
    	mainLiftMotor = new TalonSRX(RobotMap.Lift.RIGHT_WINCH_MOTOR);
    	mainLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    	followerLiftMotor = new TalonSRX(RobotMap.Lift.LEFT_WINCH_MOTOR);
//    	followerLiftMotor.set(ControlMode.Follower, RobotMap.Lift.LEFT_WINCH_MOTOR);
    	mainLiftMotor.configNominalOutputForward(0, 30);
		mainLiftMotor.configNominalOutputReverse(0, 30);
		mainLiftMotor.configPeakOutputForward(1.0, 30);
		mainLiftMotor.configPeakOutputReverse(-1.0, 30);
		followerLiftMotor.configNominalOutputForward(0, 30);
		followerLiftMotor.configNominalOutputReverse(0, 30);
		followerLiftMotor.configPeakOutputForward(1.0, 30);
		followerLiftMotor.configPeakOutputReverse(-1.0, 30);
    	botSecondStageHal = new DigitalInput(RobotMap.Lift.SECOND_STAGE_HAL_BOT);
    	topSecondStageHal = new DigitalInput(RobotMap.Lift.SECOND_STAGE_HAL_TOP);
    	botCarriageHal = new DigitalInput(RobotMap.Lift.CARRIAGE_HAL_BOT);
    	topCarriageHal = new DigitalInput(RobotMap.Lift.CARRIAGE_HAL_TOP);
    }
    public double getLiftHeight(){
    	double height = 0;
    	if(getRawLift()/LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT > LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT){
    		height += LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT;
    		height += (getRawLift()-LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT*LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT)/LiftPIDConstants.CARRIAGE_STAGE_TRANSLATION_CONSTANT;
    	}else{
    		height += getRawLift()/LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT;
    	}
    	return height;
    }
    public double getRawLift(){
    	return (double) mainLiftMotor.getSensorCollection().getQuadraturePosition();
    }
    
    public double getLiftDistance() {
    	return (getRawLift()/4096.0 * 1.9 * Math.PI);
    }
    public void setLift(double power){
    	mainLiftMotor.set(ControlMode.PercentOutput, power);
    	followerLiftMotor.set(ControlMode.PercentOutput, -power);
    }
    
    public int getLiftState(){
    	if(isSecondStageAtBottom()){
    		if(isCarriageAtBottom()){
    			SmartDashboard.putString("Carriage Position", "Bottom");
    			SmartDashboard.putString("Lift Position", "Bottom");
    			return LiftStateConstants.LOWEST_STATE;
    		}else if(isCarriageAtTop()){
    			SmartDashboard.putString("Carriage Position", "High");
    			SmartDashboard.putString("Lift Position", "Bottom");
    			return LiftStateConstants.CARRIAGE_HIGH_SECOND_LOW;
    		}else{
    			SmartDashboard.putString("Carriage Position", "Mid");
    			SmartDashboard.putString("Lift Position", "Bottom");
    			return LiftStateConstants.CARRIAGE_MID_SECOND_LOW;
    		}
    	}else if(isSecondStageAtTop()){
    		if(isCarriageAtBottom()){
    			SmartDashboard.putString("Carriage Position", "Bottom");
    			SmartDashboard.putString("Lift Position", "High");
    			return LiftStateConstants.CARRIAGE_HIGH_SECOND_HIGH;
    		}else if(isCarriageAtTop()){
    			SmartDashboard.putString("Carriage Position", "Top");
    			SmartDashboard.putString("Lift Position", "High");
    			return LiftStateConstants.CARRIAGE_HIGH_SECOND_HIGH;
    		}else{
    			SmartDashboard.putString("Carriage Position", "Mid");
    			SmartDashboard.putString("Lift Position", "High");
    			return LiftStateConstants.CARRIAGE_MID_SECOND_HIGH;
    		}
    	}else{
    		if(isCarriageAtBottom()){
    			SmartDashboard.putString("Carriage Position", "Bottom");
    			SmartDashboard.putString("Lift Position", "Mid");
    			return LiftStateConstants.CARRIAGE_LOW_SECOND_MID;
    		}else if(isCarriageAtTop()){
    			SmartDashboard.putString("Carriage Position", "High");
    			SmartDashboard.putString("Lift Position", "Mid");
    			return LiftStateConstants.CARRIAGE_HIGH_SECOND_MID;
    		}else{
    			SmartDashboard.putString("Carriage Position", "Mid");
    			SmartDashboard.putString("Lift Position", "Mid");
    			return LiftStateConstants.CARRIAGE_HIGH_SECOND_MID;
    		}
    	}
    }
    public boolean isCarriageAtBottom() {
    	return !botCarriageHal.get();
    }
    public boolean isSecondStageAtBottom() {
    	return !botSecondStageHal.get();
    }
    public boolean isSecondStageAtTop() {
    	return !topSecondStageHal.get();
    }
    public boolean isCarriageAtTop() {
    	return !topCarriageHal.get();
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return 0;
	}
	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		
	}
}

