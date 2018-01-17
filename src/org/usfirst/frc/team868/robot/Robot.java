/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team868.robot.commands.drive.DriveArcadeVolts;
import org.usfirst.frc.team868.robot.commands.drive.RampVoltsDrive;
import org.usfirst.frc.team868.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	//
	// Global subsystems that all commands have access to
	//
	public static final DriveSubsystem kDrive = new DriveSubsystem();
	public static final TransmissionSubsystem kTransmission = new TransmissionSubsystem();
	public static final GyroSubsystem kGyro = new GyroSubsystem();
	public static final CatcherPrototypeSubsystem kCatcher = new CatcherPrototypeSubsystem();
	public static final CollectorPrototypeSubsystem kIntake = new CollectorPrototypeSubsystem();
	public static final PowerDistributionPanel kPDP = new PowerDistributionPanel();

	//
	// Single instance of OI object (must be constructed AFTER subsystems).
	//
	public static final OI kOI = new OI();

	// Used to select auton mode
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		kDrive.setDefaultCommand(new DriveArcadeVolts());

		m_chooser.addDefault("Do Nothing", new WaitCommand(1.0));
		m_chooser.addObject("Slow Punch", RampVoltsDrive.createPunchSequence(4.0, 3.0, 1.0));
		m_chooser.addObject("Rabbit Punch", RampVoltsDrive.createPunchSequence(6.0, 1.0, 0.0));
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		commonPeriodic();
		Scheduler.getInstance().run();
	}

	/**
	 * Common things to do by the auton, teleop and disabled periodic methods.
	 */
	private void commonPeriodic() {
		kGyro.updateDashboard();
		kDrive.updateDashboard();
		kTransmission.updateDashboard();
		
		SmartDashboard.putNumber("Total Current", kPDP.getTotalCurrent());
		
		if (CollectorPrototypeSubsystem.isConnected()) {
			kIntake.updateDashboard();
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		Scheduler.getInstance().removeAll();
		Command auton = m_chooser.getSelected();

		// schedule the autonomous command (example)
		if (auton != null) {
			auton.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		commonPeriodic();
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//if (m_autonomousCommand != null) {
		//	m_autonomousCommand.cancel();
		//}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		commonPeriodic();
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
