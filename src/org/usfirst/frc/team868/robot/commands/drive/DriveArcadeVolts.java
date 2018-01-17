/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team868.robot.Robot;

/**
 * Driving in arcade mode.
 */
public class DriveArcadeVolts extends Command {
	private static final double[] kThrottleVolts = { -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, 0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 };
	private static final double[] kTurnRatio = { 1.0, 0.95, 0.90, 0.85, 0.80, 0.70, 0.60, 0.50 };
	private static final double[] kSpinVolts = { 0.0, 2.0, 2.5, 3.0, 4.0, 6.0 };
	
	public DriveArcadeVolts() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kDrive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double throttle = Robot.kOI.getThrottle();
		double steer = Robot.kOI.getSteer();
		
		if (throttle == 0.0) {
			// Spin mode
			int spinIdx = getIndex(kSpinVolts, Math.abs(steer));
			double volts = kSpinVolts[spinIdx];
			if (steer < 0) {
				Robot.kDrive.setVolts(-volts, volts);				
			} else {
				Robot.kDrive.setVolts(volts, -volts);		
			}
		} else {
			int throttleIdx = getIndex(kThrottleVolts, (throttle + 1.0) / 2.0);
			double leftVolts = kThrottleVolts[throttleIdx];
			double rightVolts = leftVolts;
			int turnIdx = getIndex(kTurnRatio, (steer + 1.0) / 2.0);
			double turnRatio = kTurnRatio[turnIdx];
			
			if (steer > 0) {
				rightVolts *= turnRatio;
			} else {
				leftVolts *= turnRatio;
			}
			Robot.kDrive.setVolts(leftVolts, rightVolts);
		}
	}

	/**
	 * Convert number in range of [0.0, 1.0] to index in array.
	 * 
	 * @param choices Array of choices (we need the length).
	 * @param ratio How far into list of choices (in range of [0.0, 1.0]).
	 * @return Index value in range of [0, array length - 1].
	 */
	private int getIndex(double[] choices, double ratio) {
		int len = choices.length;
		// Get value in range of [0.0, 
		int idx = (int) Math.floor(len * ratio);
		idx = Math.min(idx, len - 1);
		return idx;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
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
