
package org.usfirst.frc.team649.robot.util;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

enum LidarMode {
	balancedPerformanceMode,
	shortRangeMode,
	defaultRangeMode,
	maximumRangeMode,
	highSensitivityMode,
	lowSensitivityMode
};
	
public class Lidar {
	private I2C lidar;
	private boolean initialized = false;
	public static final double minDistance = 285;//temp
	public static final double maxDistance = 305;//temp
	
	public Lidar(I2C.Port port, int address) {
		lidar = new I2C(port, address);
	}
	
	private boolean writeLidar(int devAddr, int devValue)
	{
		boolean ret = lidar.write(devAddr, devValue);
		Timer.delay(0.001);
		return ret;
	}
	
	private boolean readLidar(int devAddr, int count, byte[] buffer)
	{
		byte addrBuf[] = new byte[1];
		
		addrBuf[0] = (byte) devAddr;
		
		if (lidar.writeBulk(addrBuf) == true) {
			return true;
		}
		return lidar.readOnly(buffer,  count);
	}
	
	
	public void configure(LidarMode mode) {
		
		writeLidar(0x00, 0x00);		// issue reset command
		Timer.delay(0.050);			// wait for device to reset
		
		switch (mode) {
		default:
		case balancedPerformanceMode:
			writeLidar(0x02, 0x80);		// Acquisition Count
			writeLidar(0x04, 0x08);		// Acquisition Config
			writeLidar(0x1c, 0x00);		// Threshold Bypass
			break;
			
		case shortRangeMode:
			writeLidar(0x02, 0x1d);		// Acquisition Count
			writeLidar(0x04, 0x08);		// Acquisition Config
			writeLidar(0x1c, 0x00);		// Threshold Bypass
			break;
		case defaultRangeMode:
			writeLidar(0x02, 0x80);		// Acquisition Count
			writeLidar(0x04, 0x00);		// Acquisition Config
			writeLidar(0x1c, 0x00);		// Threshold Bypass
			break;
		case maximumRangeMode:
			writeLidar(0x02, 0xff);		// Acquisition Count
			writeLidar(0x04, 0x08);		// Acquisition Config
			writeLidar(0x1c, 0x00);		// Threshold Bypass
			break;
			
		case highSensitivityMode:
			writeLidar(0x02, 0x80);		// Acquisition Count
			writeLidar(0x04, 0x08);		// Acquisition Config
			writeLidar(0x1c, 0x80);		// Threshold Bypass
			break;

		case lowSensitivityMode:
			writeLidar(0x02, 0x80);		// Acquisition Count
			writeLidar(0x04, 0x08);		// Acquisition Config
			writeLidar(0x1c, 0xb0);		// Threshold Bypass
			break;

		}
		
		this.initialized = true;
	}
	
	
	//
	// getSample() - read a sample from the sensor.
	// Returns distance in centimeters.
	//
	public double getSample() {
		byte[] inData = new byte[2];
		int sensorReading;
		
		// If the use forgot to call "configure", set to default mode.
		if (!this.initialized) {
			this.configure(LidarMode.balancedPerformanceMode);
		}
		
		// Send a "start acquisition" command to the sensor.
		if (writeLidar(0,4) == false) {
			return 0.0;
		}
				
		//if (waitReady() == false) {
		//	System.out.println("Lidar did not become ready");
		//	return 0.0;
		//}
		
		// If you set bit 7 of the device address, it will auto-increment on read.,
		// so reading 0x0F + 0x10 in one shot is accomplished by reading 0x8F.
		// If you didn't set the bit you'd get register 0x0F twice.
		
		readLidar(0x8F,  2,  inData);
		
		// Compose the two bytes into an int, then convert to double to return.
		sensorReading = ((((int) inData[0]) & 0xFF) << 8) | (((int) inData[1]) & 0xFF);
		
		
		return (double) sensorReading;
		
	}
}
