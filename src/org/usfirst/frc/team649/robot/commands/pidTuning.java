package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class pidTuning {
	if (oi.operator.PIDTunePhase()) {
		SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
	}
	
	if (isTuningPID) {
		if (oi.operator.getButton2()) {
			isTuningPID = false;
			drive.getPIDController().setPID(k_p, k_i, k_d);
			new DrivetrainPIDCommand(30).start();
		}

		if (oi.operator.getButton6()) {
			if (tuningConstant == 1) {
				k_p += 0.1;
			} else if (tuningConstant == 2) {
				k_i += 0.1;
			} else if (tuningConstant == 3) {
				k_d += 0.1;
			}
		}
		if (oi.operator.getButton4()) {
			if (tuningConstant == 1) {
				k_p -= 0.1;
			} else if (tuningConstant == 2) {
				k_i -= 0.1;
			} else if (tuningConstant == 3) {
				k_d -= 0.1;
			}
		}
		if (oi.operator.getButton5()) {
			if (tuningConstant == 1) {
				k_p += 0.01;
			} else if (tuningConstant == 2) {
				k_i += 0.01;
			} else if (tuningConstant == 3) {
				k_d += 0.01;
			}
		}
		if (oi.operator.getButton7()) {
			if (tuningConstant == 1) {
				k_p -= 0.01;
			} else if (tuningConstant == 2) {
				k_i -= 0.01;
			} else if (tuningConstant == 3) {
				k_d -= 0.01;
			}
		}
		if (oi.operator.getButton3()) {
			if (tuningConstant < 3) {
				tuningConstant += 1;
			} else {
				tuningConstant = 1;
			}
		}
		if (tuningConstant == 1) {
			SmartDashboard.putString("Currently Tuning", "k_p: " + k_p);
		} else if (tuningConstant == 2) {
			SmartDashboard.putString("Currently Tuning", "k_i: " + k_i);
		} else if (tuningConstant == 3) {
			SmartDashboard.putString("Currently Tuning", "k_d: " + k_d);
		}
		SmartDashboard.putNumber("k_p", k_p);
		SmartDashboard.putNumber("k_i", k_i);
		SmartDashboard.putNumber("k_d", k_d);
	}
	SmartDashboard.putBoolean("PID Tuning?", isTuningPID);
	SmartDashboard.putNumber("k_p", k_p);
	SmartDashboard.putNumber("k_i", k_i);
	SmartDashboard.putNumber("k_d", k_d);
}
