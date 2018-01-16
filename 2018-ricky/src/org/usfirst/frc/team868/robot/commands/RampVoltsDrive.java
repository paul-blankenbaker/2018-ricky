/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FRC team 868 (TechHOUNDS).                         */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team868.robot.Robot;

/**
 * Attempts to ramp up voltage over time on drive wheels.
 * 
 * <p>
 * WARNING: This is intended to be chained together - it does not stop motors at
 * end (unless you end at zero).
 * </p>
 */
public class RampVoltsDrive extends Command {
	// Voltage level to start left side of robot at
	private double m_leftStart;
	// Voltage level to end left side of robot at
	private double m_leftEnd;
	// Voltage level to start right side of robot at
	private double m_rightStart;
	// Voltage level to end right side of robot at
	private double m_rightEnd;
	// How long to take to ramp from starting volts to ending volts (seconds)
	private double m_time;
	// Minimum voltage to start moving
	public static final double kMinVolts = 1.5;
	// Voltage ramp rate (Volts/Sec change - a value like 12.0 means to it takes 2
	// seconds to go from -12.0 to +12.0 volts)
	public static final double kVoltRampRate = 12.0;
	// Set to true when debugging to dashboard
	private static final boolean kDebug = true;

	/**
	 * Construct a new instance with full control of both left and right side.
	 * 
	 * @param leftStart
	 *            Voltage level to start left side at.
	 * @param leftEnd
	 *            Voltage level to end left side at.
	 * @param rightStart
	 *            Voltage level to start right side at.
	 * @param rightEnd
	 *            Voltage level to end right side at.
	 * @param time
	 *            How long to take to get there (seconds).
	 */
	public RampVoltsDrive(double leftStart, double leftEnd, double rightStart, double rightEnd, double time) {
		// Use requires() here to declare subsystem dependencies
		this.m_leftStart = leftStart;
		this.m_rightStart = rightStart;
		this.m_leftEnd = leftEnd;
		this.m_rightEnd = rightEnd;
		this.m_time = time;
		requires(Robot.kDrive);
	}

	/**
	 * Construct a new instance where you want to drive straight (same rules applied
	 * to both sides).
	 * 
	 * @param start
	 *            Voltage level to start both sides at.
	 * @param end
	 *            Voltage level to end both sides at.
	 * @param time
	 *            How long to take to get there (seconds).
	 */
	public RampVoltsDrive(double start, double end, double time) {
		this(start, end, start, end, time);
	}

	/**
	 * From stopped position, speed up, drive straight and then slow down to a stop.
	 * 
	 * @param volts
	 *            Voltage level to cruise at.
	 * @param time
	 *            How long to cruise once level is reached (note, their will be
	 *            additional time to speed up and slow down).
	 * @return Command to perform the operation.
	 */
	public static Command createDriveStraight(double volts, double time) {
		double voltsMag = Math.abs(volts);

		// If very low speed, ignore ramp, just drive time at volts
		if (voltsMag <= kMinVolts) {
			return new RampVoltsDrive(volts, volts, time);
		}

		CommandGroup cg = new CommandGroup("RampDriveStraight");
		// Ramp up
		double startVolts = (volts < 0) ? -kMinVolts : kMinVolts;
		// Speed up to cruise voltage
		cg.addSequential(new RampVoltsDrive(startVolts, volts, (voltsMag - kMinVolts) / kVoltRampRate));
		// Cruise for specific amount of time
		cg.addSequential(new RampVoltsDrive(volts, volts, time));
		// Slow down to a stop
		cg.addSequential(new RampVoltsDrive(volts, 0, voltsMag / kVoltRampRate));

		return cg;
	}

	/**
	 * Builds an auton command sequence that moves the robot forward, stops for a
	 * second and then back up.
	 * 
	 * @param volts
	 *            Voltage level to cruise forward and backward at.
	 * @param cruisetime
	 *            How long to cruise.
	 * @param pauseTime
	 * 			How long to pause before backing up.
	 * @return Command that will perform the action.
	 */
	public static Command createPunchSequence(double volts, double cruisetime, double pauseTime) {
		CommandGroup cg = new CommandGroup("Punch");
		// Drive forward
		cg.addSequential(createDriveStraight(volts, cruisetime));
		// Pause (if requested)
		if (pauseTime > 0.0) {
			cg.addSequential(new WaitCommand(1.0));
		}
		// Drive backward
		cg.addSequential(createDriveStraight(-volts, cruisetime));
		return cg;
	}

	/**
	 * Sets the voltage level of the motors based on percent done of ramp time.
	 * 
	 * @param pctDone
	 *            Percentage complete (a ratio actually in the range of [0.0, 1.0].
	 */
	private void setVoltage(double pctDone) {
		double batteryVolts = RobotController.getInputVoltage();
		double left = (m_leftStart + (m_leftEnd - m_leftStart) * pctDone) / batteryVolts;
		double right = (m_rightStart + (m_rightEnd - m_rightStart) * pctDone) / batteryVolts;
		Robot.kDrive.setPower(left, right);
		if (kDebug) {
			SmartDashboard.putNumber("Battery Volts", batteryVolts);
		}
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double pctDone = Math.min(timeSinceInitialized() / m_time, 1.0);
		setVoltage(pctDone);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		Boolean done = timeSinceInitialized() >= m_time;
		return done;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// In case execute() was slightly before we determined done, set to final
		// voltage now
		setVoltage(1.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		// If we were interrupted, we will just turn off the motors
		// (maybe we should set to final voltage or do nothing as the
		// interrupter will be taking control anyway)
		Robot.kDrive.stop();
	}
}
