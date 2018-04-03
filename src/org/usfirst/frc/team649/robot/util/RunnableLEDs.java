package org.usfirst.frc.team649.robot.util;

import org.usfirst.frc.team649.robot.Robot;

public class RunnableLEDs implements Runnable{
	
	private int state;
	
	public RunnableLEDs()
	{
		
	}
	public RunnableLEDs(int stateParam)
	{
		state = stateParam;
	}
	public void run()
	{
		if (state != Robot.previousState) {
			byte[] stateArray = new byte[1];
			stateArray[0] = (byte) state;
			Robot.sp.write(stateArray, 1 );
		}
		Robot.previousState = state;
	}
}
