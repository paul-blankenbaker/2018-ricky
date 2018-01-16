/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team868.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class TimeDrive extends Command {
	private double left;
	private double right;
	private double time;
	public TimeDrive(double left, double right, double time) {
		// Use requires() here to declare subsystem dependencies
		this.left = left;
		this.right = right;
		this.time = time;
		requires(Robot.kDrive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.kDrive.setPower(left, right);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		Boolean done = timeSinceInitialized() >= time;
		return done;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.kDrive.setPower(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
