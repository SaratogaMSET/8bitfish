package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem.LiftPIDConstants;
import org.usfirst.frc.team649.robot.util.Lidar;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
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
    public static class LiftHalConstants{
    	public static int LOWEST_STATE = 1;
    	public static int CARRIAGE_LOW_SECOND_MID = 2;
    	public static int CARRIAGE_LOW_SECOND_HIGH = 3;
    	public static int CARRIAGE_MID_SECOND_HIGH = 4;
    	public static int CARRIAGE_HIGH_SECOND_HIGH = 5;
    	public static int CARRIAGE_HIGH_SECOND_MID = 6;
    	public static int CARRIAGE_HIGH_SECOND_LOW = 7;
    	public static int CARRIAGE_MID_SECOND_LOW = 8;
    }
    public static class LiftStateConstants{
    	public static final int INTAKE_EXCHANGE_STORE_STATE = 2;
    	public static final int HEADING_INTAKE_EXCHANGE_STORE_STATE = 1;
    	public static final int SWITCH_STATE = 4;
    	public static final int HEADING_SWITCH_STATE = 3;
    	public static final int LOW_SCALE_STATE = 6;
    	public static final int HEADING_LOW_SCALE_STATE = 5;
    	
    	public static final int HEADING_LOW_MID_SCALE = 16;
    	public static final int LOW_MID_SCALE = 17;
    	
    	public static final int MID_SCALE_STATE = 8;
    	public static final int HEADING_MID_SCALE_STATE = 7;
    	public static final int HIGH_SCALE_STATE = 10;
    	public static final int HEADING_HIGH_SCALE_STATE = 9;
    	public static final int CUSTOM_STATE = 12;
    	public static final int HEADING_CUSTOM_STATE_UP = 13;
    	public static final int HEADING_CUSTOM_STATE_DOWN = 11;
    	public static final int HEADING_INTAKE_2 = 14;
    	public static final int INTAKE_2 = 15;
    	
    }
    public static class LiftEncoderConstants{
    	public static int LOW_STATE = 0;
    	public static int SWITCH_STATE = 20000; // 20000 P
    	public static int LOW_SCALE_STATE = 32000; // 35500 P
    	public static int MID_SCALE_STATE = 44400; // 46000 P
    	public static int HIGH_SCALE_STATE = 44400; // 48100 P
    	public static int ADJ_DIST = 4000;
    	public static int INTAKE_2_STATE = 12500;
    	public static int LOW_MID_SCALE = 38000;
    }
    public static class LiftConstants{
    	public static double unitsPerCmSecond = 268;
    	public static double unitsPerCmCarriage = 244;
    	public static double maxSecondHeight = 93;
    	public static double flipUpperPos = 25;
    	public static double absTol = 15;
    }
    public TalonSRX mainLiftMotor,followerLiftMotor,followerLiftMotor2;
    public DigitalInput botSecondStageHalRight, topSecondStageHalRight, botCarriageHalRight, topCarriageHalRight, botSecondStageHalLeft, topSecondStageHalLeft, botCarriageHalLeft, topCarriageHalLeft;
    double liftPIDOutPut;
    
    public LiftSubsystem(){
    	super(LiftPIDConstants.k_P, LiftPIDConstants.k_I, LiftPIDConstants.k_D);
    	mainLiftMotor = new TalonSRX(RobotMap.Lift.RIGHT_WINCH_MOTOR);
    	mainLiftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Robot.timeoutMs);
    	mainLiftMotor.setInverted(false); // true on final bot
    	mainLiftMotor.setSensorPhase(true);
    	followerLiftMotor = new TalonSRX(RobotMap.Lift.LEFT_WINCH_MOTOR);
    	followerLiftMotor.setInverted(true); // false on f bot
    	followerLiftMotor.set(ControlMode.Follower, RobotMap.Lift.RIGHT_WINCH_MOTOR);
    	
    	mainLiftMotor.configNominalOutputForward(0, Robot.timeoutMs);
		mainLiftMotor.configNominalOutputReverse(0, Robot.timeoutMs);
		mainLiftMotor.configPeakOutputForward(1.0, Robot.timeoutMs);
		mainLiftMotor.configPeakOutputReverse(-1.0, Robot.timeoutMs);
		followerLiftMotor.configNominalOutputForward(0, Robot.timeoutMs);
		followerLiftMotor.configNominalOutputReverse(0, Robot.timeoutMs);
		followerLiftMotor.configPeakOutputForward(1.0, Robot.timeoutMs);
		followerLiftMotor.configPeakOutputReverse(-1.0, Robot.timeoutMs);
		mainLiftMotor.setNeutralMode(NeutralMode.Brake);
		followerLiftMotor.setNeutralMode(NeutralMode.Brake);
		mainLiftMotor.configMotionCruiseVelocity(3200, Robot.timeoutMs);
		mainLiftMotor.configMotionAcceleration(3450, Robot.timeoutMs); // 400 actual
		mainLiftMotor.selectProfileSlot(0, 0);
		//0.3197
		mainLiftMotor.config_kF(0, 0.3197, Robot.timeoutMs);
		mainLiftMotor.config_kP(0, 3.5, Robot.timeoutMs);
		mainLiftMotor.config_kI(0, 0.015, Robot.timeoutMs);
		mainLiftMotor.config_kD(0, 0.07, Robot.timeoutMs);
    	botSecondStageHalRight = new DigitalInput(RobotMap.Lift.SECOND_STAGE_HAL_BOT_RIGHT);
    	topSecondStageHalRight = new DigitalInput(RobotMap.Lift.SECOND_STAGE_HAL_TOP_RIGHT);
    	botCarriageHalRight = new DigitalInput(RobotMap.Lift.CARRIAGE_HAL_BOT_RIGHT);
    	topCarriageHalRight = new DigitalInput(RobotMap.Lift.CARRIAGE_HAL_TOP_RIGHT);
    
    	botSecondStageHalLeft = new DigitalInput(RobotMap.Lift.SECOND_STAGE_HAL_BOT_LEFT);
    	topSecondStageHalLeft = new DigitalInput(RobotMap.Lift.SECOND_STAGE_HAL_TOP_LEFT);
    	botCarriageHalLeft = new DigitalInput(RobotMap.Lift.CARRIAGE_HAL_BOT_LEFT);
    	topCarriageHalLeft = new DigitalInput(RobotMap.Lift.CARRIAGE_HAL_TOP_LEFT);
    }
//    public double getLidarValue(){
//    	return lidar.getSample()
    ;
//    }
    //u have a max height of 2nd stage and u have a translsational value tuned by max heihgt/winch turns. 
    //Then u get total height by lidar + (total winch - lidar portion converted)

    public double getLiftHeight(){
    	return 0.0;
    }

    public double getLiftHeightNoLidar(){
    	double height = 0;
    	if(getRawLift()/LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT > LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT){
    		height += LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT;
    		height += (getRawLift()-LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT*LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT)/LiftPIDConstants.CARRIAGE_STAGE_TRANSLATION_CONSTANT;
    	}else{
    		height += getRawLift()/LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT;
    	}
    	return height;
    }
    public double getInchesToRaw(double inches){
    	if(inches > LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT){
    		double dist =  LiftPIDConstants.MAX_SECOND_STAGE_HEIGHT*LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT;
    		dist += (inches - dist) * LiftPIDConstants.CARRIAGE_STAGE_TRANSLATION_CONSTANT;
    		return dist;
    	}else{
    		return inches*LiftPIDConstants.SECOND_STAGE_TRANSLATION_CONSTANT;
    	}
    }
    public double getRawLift(){
    	return (double) mainLiftMotor.getSensorCollection().getQuadraturePosition();
    }
    public double getCarriageHeight(){
    	return (getRawLift() - Robot.lidarValue * LiftConstants.unitsPerCmSecond)/LiftConstants.unitsPerCmCarriage;
    }
    public boolean canFlip(){
    	return getCarriageHeight()<LiftSubsystem.LiftConstants.flipUpperPos;
    }
    public double getLiftDistance() {
    	return (getRawLift()/4096.0 * 1.9 * Math.PI);
    }
    public void setLift(double power){
    	mainLiftMotor.set(ControlMode.PercentOutput, power);
    }
    public void setLiftMotion(int pos){
    	mainLiftMotor.set(ControlMode.MotionMagic, pos);
    	SmartDashboard.putBoolean("Is Motion Profiling", true);
    }
    public void setLiftWithSafety(double power, boolean override) {
    	if (!override) {	
    		if (isCarriageAtBottom() && isSecondStageAtBottom()) {
    			if(power < 0) {
    				power = 0;
    			}
    		} else if (isCarriageAtTop() && isSecondStageAtTop()) {
    			if (power > 0) {
    				power = 0;
    			}
    		}
    		setLift(power);
    	} else if (override) {
    		setLift(power);
    	}
    }
    public void resetLiftEncoder(){
    	mainLiftMotor.setSelectedSensorPosition(0, 0, 30);
    }

    public int getLiftState(){
    	if(isSecondStageAtBottom()){
    		if(isCarriageAtBottom()){
    			SmartDashboard.putString("Carriage Position", "Bottom");
    			SmartDashboard.putString("Lift Position", "Bottom");
    			return LiftHalConstants.LOWEST_STATE;
    		}else if(isCarriageAtTop()){
    			SmartDashboard.putString("Carriage Position", "High");
    			SmartDashboard.putString("Lift Position", "Bottom");
    			return LiftHalConstants.CARRIAGE_HIGH_SECOND_LOW;
    		}else{
    			SmartDashboard.putString("Carriage Position", "Mid");
    			SmartDashboard.putString("Lift Position", "Bottom");
    			return LiftHalConstants.CARRIAGE_MID_SECOND_LOW;
    		}
    	}else if(isSecondStageAtTop()){
    		if(isCarriageAtBottom()){
    			SmartDashboard.putString("Carriage Position", "Bottom");
    			SmartDashboard.putString("Lift Position", "High");
    			return LiftHalConstants.CARRIAGE_HIGH_SECOND_HIGH;
    		}else if(isCarriageAtTop()){
    			SmartDashboard.putString("Carriage Position", "Top");
    			SmartDashboard.putString("Lift Position", "High");
    			return LiftHalConstants.CARRIAGE_HIGH_SECOND_HIGH;
    		}else{
    			SmartDashboard.putString("Carriage Position", "Mid");
    			SmartDashboard.putString("Lift Position", "High");
    			return LiftHalConstants.CARRIAGE_MID_SECOND_HIGH;
    		}
    	}else{
    		if(isCarriageAtBottom()){
    			SmartDashboard.putString("Carriage Position", "Bottom");
    			SmartDashboard.putString("Lift Position", "Mid");
    			return LiftHalConstants.CARRIAGE_LOW_SECOND_MID;
    		}else if(isCarriageAtTop()){
    			SmartDashboard.putString("Carriage Position", "High");
    			SmartDashboard.putString("Lift Position", "Mid");
    			return LiftHalConstants.CARRIAGE_HIGH_SECOND_MID;
    		}else{
    			SmartDashboard.putString("Carriage Position", "Mid");
    			SmartDashboard.putString("Lift Position", "Mid");
    			return LiftHalConstants.CARRIAGE_HIGH_SECOND_MID;
    		}
    	}
    }
    public double getRawLiftVel(){
    	return mainLiftMotor.getSelectedSensorVelocity(0);
    }
    public boolean isCarriageAtBottom() {
    	return !botCarriageHalRight.get() || !botCarriageHalLeft.get();
    }
    public boolean isSecondStageAtBottom() {
    	return !botSecondStageHalRight.get() || !botSecondStageHalLeft.get();
    }
    public boolean isSecondStageAtTop() {
    	return !topSecondStageHalRight.get() || !topSecondStageHalLeft.get() ;
    }
    public boolean isCarriageAtTop() {
    	return !topCarriageHalRight.get() || !topCarriageHalLeft.get();
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return getRawLift();
	}
	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		
	}
}