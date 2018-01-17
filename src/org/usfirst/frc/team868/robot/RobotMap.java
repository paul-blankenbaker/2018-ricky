/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot;

/**
 * Used to track allocation of resources on roboRIO and CAN bus.
 */
public final class RobotMap {

	/**
	 * roboRIO PWM usage.
	 */
	public class PWM {
		public static final int kDriveRightMotors = 8;
		public static final int kDriveLeftMotors = 9;
	}

	/**
	 * roboRIO DIO usage.
	 */
	public class DIO {
		public static final int kDriveLeftChA = 1;
		public static final int kDriveLeftChB = 2;
		public static final int kDriveRightChA = 3;
		public static final int kDriveRightChB = 4;
	}
	
	/**
	 * PCM solenoid usage.
	 */
	public class PCM {
		public static final int kDriveShifter = 0;
		
		// NOTE: Catcher is wooden prototype that squeeze and punches
		// (can not coexist with Collector)
		public static final int kCatcherGrabber = 5;
		public static final int kCatcherPuncher = 7;

		// NOTE: Collector is wheeled collector that can squeeze
		// (can not coexist with Catcher)
		public static final int kCollectorGrabber = 5;
	}
}
