package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Servo;

public class HangSubsystem {
	public Servo servo;
	
	public HangSubsystem()
	{
		servo = new Servo(RobotMap.Hang.hangport);
	}
	
	public void grabHook()
	{
		servo.setAngle(180);
	}
	
	public void resetHook()
	{
		servo.setAngle(0);
	}
	
}
