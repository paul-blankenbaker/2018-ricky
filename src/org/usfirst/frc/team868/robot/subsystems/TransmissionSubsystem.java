/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot.subsystems;

import org.usfirst.frc.team868.robot.Robot;
import org.usfirst.frc.team868.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The subsystem used to manage the two speed transmission.
 */
public final class TransmissionSubsystem extends Subsystem {
	
	// Set to true to enable more dashboard output
	private static final boolean kDebug = true;
	
	/**
	 * Solenoid used when changing gears.
	 */
	private final Solenoid shifter;

	/**
	 * State of shifter solenoid when in high gear.
	 */
	private static final boolean kHighGearState = false;
	
	/**
	 * Current state of transmission (cached to avoid JNI calls).
	 */
	private boolean m_gearState;
	
	/**
	 * Maximum speed in inches/sec in low gear before auto-shifting up (MUST HAVE GAP TO PREVENT THRASHING).
	 */
	private static final double kMaxLowGearSpeed = (8 * 12);
	
	/**
	 * Minimum speed in inches/sec in high gear before auto-shifting down (MUST HAVE GAP TO PREVENT THRASHING).
	 */
	private static final double kMinHighGearSpeed = kMaxLowGearSpeed / 2.0;

	/**
	 * Constructs an instance of the two speed transmission - only do this once.
	 */
	public TransmissionSubsystem() {
		super("Transmission");

		// Initialize gear shifter
		shifter = new Solenoid(RobotMap.PCM.kDriveShifter);
		m_gearState = (shifter.get() == kHighGearState);

		if (kDebug) {
			// When debugging enabled, put subsystem to dashboard to
			// see what command is running on it
			SmartDashboard.putData("Transmission", this);
		}
	}

	public void initDefaultCommand() {
		// NOTE: Don't do this here - leads to construction nightmare!
	}
	
	/**
	 * Get state of two speed transmission.
	 * 
	 * @return true if in high gear, false if in low gear.
	 */
	public boolean inHighGear() {
		// return cached value
		return m_gearState;
	}
	
	/**
	 * Shift two speed transmission into high gear.
	 */
	public void shiftToHighGear() {
		m_gearState = kHighGearState;
		shifter.set(m_gearState);
	}
	
	/**
	 * Shift two speed transmission into low gear.
	 */
	public void shiftToLowGear() {
		m_gearState = !kHighGearState;
		shifter.set(m_gearState);
	}
	
	/**
	 * Shift two speed transmission automatically.
	 */
	public void shiftAutomatic() {
		double speed = Robot.kDrive.getSpeed();
		if (m_gearState == kHighGearState) {
			if (speed < kMinHighGearSpeed) {
				// In high gear and speed has dropped below threshold (downshift)
				shiftToLowGear();
			}
		} else {
			if (speed > kMaxLowGearSpeed) {
				// In low gear and speed has exceeded threshold (upshift)
				shiftToHighGear();
			}
		}
	}

	/**
	 * Helper method to provide some diagnostic output (set internal kDebug constant
	 * to true to enable more).
	 */
	public void updateDashboard() {
		SmartDashboard.putBoolean("High Gear", inHighGear());
	}

}
