package org.usfirst.frc.team649.robot.util;

import org.usfirst.frc.team649.robot.Robot;

public class RunnableLEDs implements Runnable {

	private int previousState;

	public RunnableLEDs() {
		previousState = 0;
	}

	public void run() {
		if (previousState != Robot.ledState) {
			byte[] stateArray = new byte[1];
			stateArray[0] = (byte) previousState;
			if (Robot.sp != null) {
				Robot.sp.write(stateArray, 1);
			}
		}
		previousState = Robot.ledState;
	}
}
