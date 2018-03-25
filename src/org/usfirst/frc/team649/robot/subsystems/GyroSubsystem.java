package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GyroSubsystem extends PIDSubsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public static class GyroPIDConstants {
		public static double k_p = 0.4;//0.9;//0.02;
		public static double k_i = 0;
		public static double k_d = 0.9;//0.042;
		
		public static final double GYRO_ABS_TOLERANCE = 2;
    }
	
	public double GyroPIDOutput;
    public boolean drivingStraight;
	public ADXRS450_Gyro gyro;

	public GyroSubsystem() {
		super("Gyro Subsystem", GyroPIDConstants.k_p, GyroPIDConstants.k_i, GyroPIDConstants.k_d);
		drivingStraight = false;
		gyro = new ADXRS450_Gyro();
		GyroPIDOutput = 0;
		this.getPIDController().setAbsoluteTolerance(GyroPIDConstants.GYRO_ABS_TOLERANCE);
		this.getPIDController().setOutputRange(-1, 1);
	}
	public double getP() {
		return GyroPIDConstants.k_p;
	}
	public double getI() {
		return GyroPIDConstants.k_i;
	}
	public double getD() {
		return GyroPIDConstants.k_d;
	}
	public void setP(double d) {
		GyroPIDConstants.k_p += d;
	}
	public void setI(double d) {
		GyroPIDConstants.k_i += d;
	}
	public void setD(double d) {
		GyroPIDConstants.k_d += d;
	}
	 public double getGyroAngle() {
	    	return gyro.getAngle();
	 }
	    
	 public void resetGyro() {
	    	gyro.reset();
	 }
	    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return gyro.getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		if(drivingStraight) {
			GyroPIDOutput = output;
//			double left;
//			double right;
//			if (output > 0) {
//				left = 1.0;
//				right = 1.0 - output;
//			} else if (output < 0) {
//				right = 1.0 + output;
//				left = 1.0;
//			} else {
//				right = 1.0;
//				left = 1.0;
//			}
//			Robot.drive.rawDrive(left, right);
		} else {
			Robot.drive.rawDrive(-output, -output);
		}
	}
	public void setDrivingStraight(boolean straight) {
		drivingStraight = straight;
	}
	
	public double getGyroPIDOutput() {
		return GyroPIDOutput;
	}
}

