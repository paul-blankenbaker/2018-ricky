/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot.commands.transmission;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team868.robot.Robot;

/**
 * Command to shift into low or high gear.
 */
public class ShiftGears extends Command {
	
	private boolean shiftIntoHigh;
	private boolean waitUntilInterrupted;

	/**
	 * Constructs a new instance of the shift gears command.
	 * 
	 * @param highGear Pass true to shift into high gear, false to shift into low gear.
	 * @param untilInterrupted Pass true to keep running this command until someone interrupts us.
	 */
	public ShiftGears(boolean highGear, boolean untilInteruppted) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kTransmission);
		shiftIntoHigh = highGear;
		waitUntilInterrupted = untilInteruppted;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (shiftIntoHigh) {
			Robot.kTransmission.shiftToHighGear();
		} else {
			Robot.kTransmission.shiftToLowGear();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return !waitUntilInterrupted;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
