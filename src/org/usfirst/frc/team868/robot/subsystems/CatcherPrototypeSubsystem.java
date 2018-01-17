package org.usfirst.frc.team868.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team868.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Temporary prototype collector that has two arms that can squeeze cubes and
 * one puncher to push them out.
 */
public class CatcherPrototypeSubsystem extends Subsystem {
	private Solenoid catchBar;
	private Solenoid puncher;
	
	/**
	 * NOTE: If you set this to true, make sure you set Collector's flag to false - only one can be connected!
	 */
	private static final boolean isConnected = false;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public CatcherPrototypeSubsystem() {
		if (isConnected) {
			catchBar = new Solenoid(RobotMap.PCM.kCatcherGrabber);
			puncher = new Solenoid(RobotMap.PCM.kCatcherPuncher);
		}
	}

	public void open() {
		if (isConnected) {
			catchBar.set(false);
			puncher.set(true);
		}
	}

	public void close() {
		if (isConnected) {
			catchBar.set(true);
			puncher.set(true);
		}
	}

	public void push() {
		if (isConnected) {
			puncher.set(false);
			catchBar.set(false);
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
