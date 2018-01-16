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
 * Pulls in cube by spinning wheels on arms.
 */
public class SuckInCube extends Command {
	public SuckInCube() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kIntake);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.kIntake.spinIn();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// Stop spinning if current spike
		return (Robot.kIntake.getSpinCurrent() > 10.0);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.kIntake.spinStop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
