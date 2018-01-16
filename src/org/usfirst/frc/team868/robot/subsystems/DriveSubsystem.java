/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot.subsystems;

import org.usfirst.frc.team868.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The subsystem used to manage sensors and motors associated with the drive
 * train.
 */
public final class DriveSubsystem extends Subsystem {
	private Spark rightM;
	private Spark leftM;
	private Encoder rightE;
	private Encoder leftE;

	// Set to true to enable more dashboard output
	private static final boolean kDebug = true;

	// Related to setting up encoders
	private static final double kWheelDiameter = 4;
	private static final int kEncSamplesToAvg = 8;
	private static final double kPulsesPerRotation = 4096 / kEncSamplesToAvg;

	/**
	 * Constructs an instance of the drive train - only do this once.
	 */
	public DriveSubsystem() {
		super("Drive");

		leftM = new Spark(RobotMap.PWM.kDriveLeftMotors);
		rightM = new Spark(RobotMap.PWM.kDriveRightMotors);

		// TODO: Need encoder ports
		leftE = createEncoder("Left", RobotMap.DIO.kDriveLeftChA, RobotMap.DIO.kDriveLeftChB, false);
		rightE = createEncoder("Right", RobotMap.DIO.kDriveRightChA, RobotMap.DIO.kDriveRightChB, false);

		// Let's group things in test mode
		leftM.setName("Drive", "LeftMotor");
		rightM.setName("Drive", "RightMotor");

		if (kDebug) {
			// When debugging enabled, put subsystem to dashboard to
			// see what command is running on it
			SmartDashboard.putData("Drive", this);
		}
	}

	/**
	 * Helper method to initialize encoder.
	 * 
	 * @param label
	 *            Label to associate with encoder.
	 * @param chA
	 *            DIO port on roboRIO for A channel.
	 * @param chB
	 *            DIO port on roboRIO for B channel.
	 * @param invert
	 *            Pass true if direction needs to be inverted.
	 * @return New Encoder object to measure drive train movement.
	 */
	private Encoder createEncoder(String label, int chA, int chB, boolean invert) {
		Encoder e = new Encoder(chA, chB, invert, Encoder.EncodingType.k4X);
		e.setName("Drive", label + "Encoder");

		// From WPILib screen steps, lets reduce jitter a bit as we don't need
		// sub-millimeter accuracy
		// Seconds that must elapse with no count changes to consider wheel stopped
		e.setMaxPeriod(0.05);
		// Inches traveled by each pulse
		e.setDistancePerPulse(kWheelDiameter * Math.PI / kPulsesPerRotation);
		// When movement is below 0.5 in/sec, consider robot stopped
		e.setMinRate(0.5);
		// How many samples to average to reduce jitter (also reduces accuracy)
		e.setSamplesToAverage(kEncSamplesToAvg);
		return e;
	}

	public void initDefaultCommand() {
		// NOTE: Don't do this here - leads to construction nightmare!
	}

	/**
	 * Set power on left and right side.
	 * 
	 * @param left
	 *            Power setting for left side in range of [-1.0, +1.0] where
	 *            positive moves forward.
	 * @param right
	 *            Power setting for right side in range of [-1.0, +1.0] where
	 *            positive moves forward.
	 */
	public void setPower(double left, double right) {
		right = -right;
		leftM.set(left);
		rightM.set(right);
	}

	/**
	 * Stops all motors on drive train.
	 */
	public void stop() {
		setPower(0, 0);
	}

	/**
	 * Gets the current power setting for the right side motors.
	 * 
	 * @return A value in the range of [-1.0, +1.0] where positive indicates moving
	 *         forward.
	 */
	public double getPowerRight() {
		double power = -rightM.get();
		return power;
	}

	/**
	 * Gets the current power setting for the left side motors.
	 * 
	 * @return A value in the range of [-1.0, +1.0] where positive indicates moving
	 *         forward.
	 */
	public double getPowerLeft() {
		double power = -leftM.get();
		return power;
	}

	/**
	 * Helper method to provide some diagnostic output (set internal kDebug constant
	 * to true to enable more).
	 */
	public void updateDashboard() {
		SmartDashboard.putNumber("Drive Distance", getDistance());
		SmartDashboard.putNumber("Drive Speed", getSpeed());
		if (kDebug) {
			SmartDashboard.putNumber("Left Distance", getDistanceLeft());
			SmartDashboard.putNumber("Right Distance", getDistanceRight());
			SmartDashboard.putNumber("Left Power", getPowerLeft());
			SmartDashboard.putNumber("Right Power", getPowerRight());
		}
	}

	/**
	 * Returns the average speed of the two encoders.
	 * 
	 * @return Speed in inches/second.
	 */
	public double getSpeed() {
		double speed = (leftE.getRate() + rightE.getRate()) / 2;
		return speed;
	}

	/**
	 * Get the average distance traveled since time of construction.
	 * 
	 * @return Distance in inches.
	 */
	public double getDistance() {
		double dist = (getDistanceLeft() + getDistanceRight()) / 2.0;
		return dist;
	}

	/**
	 * Get the distance traveled by left side of robot frame since time of
	 * construction.
	 * 
	 * @return Distance in inches.
	 */
	public double getDistanceLeft() {
		return leftE.getDistance();
	}

	/**
	 * Get the distance traveled by right side of robot frame since time of
	 * construction.
	 * 
	 * @return Distance in inches.
	 */
	public double getDistanceRight() {
		// TODO Auto-generated method stub
		return 0;
	}

}
