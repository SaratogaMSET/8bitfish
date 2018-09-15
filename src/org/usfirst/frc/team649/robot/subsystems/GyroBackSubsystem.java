package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GyroBackSubsystem extends PIDSubsystem {

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

	public GyroBackSubsystem() {
		super("Gyro Back Subsystem", GyroPIDConstants.k_p, GyroPIDConstants.k_i, GyroPIDConstants.k_d);

		this.getPIDController().setAbsoluteTolerance(GyroPIDConstants.GYRO_ABS_TOLERANCE);
		this.getPIDController().setOutputRange(-1, 1);
	}
	    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return -Robot.gyro.gyro.getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		if(drivingStraight) {
			GyroPIDOutput = output;
		} else {
			if (this.getPIDController().getSetpoint() < 0) {
				Robot.drive.rawDrive(0, output);
			} else {
				Robot.drive.rawDrive(output, 0);
			}
		}
	}
	public void setDrivingStraight(boolean straight) {
		drivingStraight = straight;
	}
	
	public double getGyroPIDOutput() {
		return GyroPIDOutput;
	}
}

