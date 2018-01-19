package org.usfirst.frc.team649.autonomous;

import edu.wpi.first.wpilibj.DriverStation;

public class autoMaster {
	// before its done, verify how the alliance color data is received-- not sure if
	// toString will correctly return as a
	// string or not, or if Alliance is some sort of weird number data

	public static String gameData;
	public static String AllianceColor;
	public static String blueAlliance = "blue";
	public static String redAlliance = "red";

	// if false, it's left, if true, it's right
	public static boolean allySwitch = false;
	public static boolean scale = false;
	public static boolean opponentSwitch = false;

	public static void autoDecider() {
//		gameData = DriverStation.getInstance().getGameSpecificMessage();
//		AllianceColor = DriverStation.getInstance().getAlliance().toString();
//		if (AllianceColor.equalsIgnoreCase(blueAlliance)) {
//			if (gameData.charAt(0) == 'L') {
//				// blue alliance, your switch plate Left side
//				allySwitch = false;
//			} else {
//				// blue alliance, your switch plate right side
//				allySwitch = true;
//			}
//			if (gameData.charAt(1) == 'L') {
//				// blue alliance, scale plate left side
//				scale = false;
//			} else {
//				// blue alliance, scale plate right side
//				scale = true;
//			}
//			if (gameData.charAt(2) == 'L') {
//				// blue alliance, opponent switch plate left side
//				opponentSwitch = false;
//			} else {
//				// blue alliance, opponent switch plate right side
//				opponentSwitch = true;
//			}
//		}
//		if (AllianceColor.equalsIgnoreCase(redAlliance)) {
//			if (gameData.charAt(0) == 'L') {
//				// red alliance, your switch plate Left side
//				allySwitch = false;
//			} else {
//				// red alliance, your switch plate right side
//				allySwitch = true;
//			}
//			if (gameData.charAt(1) == 'L') {
//				// red alliance, scale plate left side
//				scale = false;
//			} else {
//				// red alliance, scale plate right side
//				scale = true;
//			}
//			if (gameData.charAt(2) == 'L') {
//				// red alliance, opponent switch plate left side
//				opponentSwitch = false;
//			} else {
//				// red alliance, opponent switch plate right side
//				opponentSwitch = true;
//			}
//		}
//	}
//
//	public static void runAuto() {
//		if (AllianceColor.equalsIgnoreCase(blueAlliance)) {
//			if (allySwitch == true) {
//				// blue alliance switch on right side
//				if (scale == true) {
//					// blue alliance switch on right side, scale on right side
//					if (opponentSwitch == true) {
//						// blue alliance switch on right side, scale on right side, opponent switch on
//						// right side
//
//					} else if (opponentSwitch == false) {
//						// blue alliance switch on right side, scale on right side, opponentSwitch on
//						// left side
//
//					}
//				} else if (scale == false) {
//					// blue alliance switch on right side, scale on left side
//					if (opponentSwitch == true) {
//						// blue alliance switch on right side, scale on left side, opponent switch on
//						// right side
//					} else if (opponentSwitch == false) {
//						// blue alliance switch on right side, scale on left side, opponent switch on
//						// left side
//					}
//				}
//			}
//			if (allySwitch == false) {
//				// blue alliance switch on left side
//				if (scale == true) {
//					// blue alliance switch on left side, scale on right side
//					if (opponentSwitch == true) {
//						// blue alliance switch on left side, scale on right side, opponent switch on
//						// right side
//					} else if (opponentSwitch == false) {
//						// blue alliance switch on left side, scale on right side, opponent switch on
//						// left side
//					}
//				} else if (scale == false) {
//					// blue alliance switch on left side, scale on left side
//					if (opponentSwitch == true) {
//						// blue alliance switch on left side, scale on left side, opponent switch on
//						// right side
//					} else if (opponentSwitch == false) {
//						// blue alliance switch on left side, scale on left side, opponent switch on
//						// left side
//					}
//				}
//			}
//		}
//		if (AllianceColor.equalsIgnoreCase(redAlliance)) {
//			if (allySwitch == true) {
//				// red alliance, switch on right side
//				if (scale == true) {
//					// red alliance, switch on right side, scale on right side
//					if (opponentSwitch == true) {
//						// red alliance, switch on right side, scale on right side, opponent switch on
//						// right side
//
//					} else if (opponentSwitch == false) {
//						// red alliance, switch on right side, scale on right side, opponent switch on
//						// left side
//
//					}
//				} else if (scale == false) {
//					// red alliance, scale on left side
//					if (opponentSwitch == true) {
//						// red alliance switch on right side, scale on left side, opponent switch on
//						// right side
//
//					} else if (opponentSwitch == false) {
//						// red alliance, switch on right side, scale on left side, opponent switch on
//						// left side
//					}
//				}
//			}
//			if (allySwitch == false) {
//				// red alliance, switch on left side
//				if (scale == true) {
//					// red alliance, switch on left side, scale on right side
//					if (opponentSwitch == true) {
//						// red alliance, switch on left side, scale on right side, opponent switch on
//						// right side
//					} else if (opponentSwitch == false) {
//						// red alliance, switch on left side, scale on right side, opponent switch on
//						// left side
//					}
//				} else if (scale == false) {
//					// red alliance, switch on left side, scale on left side
//					if (opponentSwitch == true) {
//						// red alliance, switch on left side, scale on left side, opponent switch on
//						// right side
//					} else if (opponentSwitch == false) {
//						// red alliance, switch on left side, scale on right side, opponent switch on
//						// left side
//					}
//				}
//			}
//		}
	}
}
