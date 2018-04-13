package org.usfirst.frc.team649.autonomous;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class MotionProfileTrajectoryManager {
	/**
	 * Saves a Trajectory to a CSV file
	 * @param t - the trajectory to export
	 * @param autoPathName - the name of the auto path. DO NOT INCLUDE the .csv extension.
	 */
	public static void saveTrajectory(Trajectory t, String autoPathName) {
		if(t.equals(null)) {
			return;
		}
		if(!autoPathName.contains(".csv")) {
			autoPathName += ".csv";
		}
		Pathfinder.writeToCSV(new File("./motionprofile-csv-trajectories/" + autoPathName), t);
	}
	
	/**
	 * Reads a Trajectory from a CSV file.
	 * @param autoPathName - the name of the auto path. DO NOT INCLUDE the .csv extension.
	 * @return the pregenerated Trajectory for the given auto path
	 */
	public static Trajectory trajectoryForName(String autoPathName) {
		return Pathfinder.readFromCSV(new File("./motionprofile-csv-trajectories/" + autoPathName));
	}
}
