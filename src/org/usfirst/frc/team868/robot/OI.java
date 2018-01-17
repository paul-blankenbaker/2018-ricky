/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot;

import org.usfirst.frc.team868.robot.commands.catcher.CloseArm;
import org.usfirst.frc.team868.robot.commands.catcher.OpenArm;
import org.usfirst.frc.team868.robot.commands.catcher.PushCube;
import org.usfirst.frc.team868.robot.commands.collector.GrabCube;
import org.usfirst.frc.team868.robot.commands.collector.SuckInCube;
import org.usfirst.frc.team868.robot.commands.collector.ThrowCube;
import org.usfirst.frc.team868.robot.commands.transmission.AutoShift;
import org.usfirst.frc.team868.robot.commands.transmission.ShiftGears;
import org.usfirst.frc.team868.robot.subsystems.CollectorPrototypeSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public final class OI {
	// Percent movement of axis that needs to be exceeded to be considered off zero.
	private static final double kDeadZone = 0.05;
	private XboxController m_driver;

	public OI() {
		m_driver = new XboxController(0);

		setupSmartDashboard();
		setupDriver();
	}

	private void setupDriver() {
		// For manual shifting, only drop to low gear while button is pressed down
		Button manualShift = new JoystickButton(m_driver, 6);
		manualShift.whenPressed(new ShiftGears(false, false));
		manualShift.whenReleased(new ShiftGears(true, false));

		// Toggle auto shifting (note, driver may need to press/release manual shifting
		// to get back to high gear)
		Button autoShift = new JoystickButton(m_driver, 7);
		autoShift.toggleWhenPressed(new AutoShift());
	}

	private void setupSmartDashboard() {
		// Collector prototype commands (not always connected to robot)
		if (CollectorPrototypeSubsystem.isConnected()) {
			SmartDashboard.putData("Close Arms", new GrabCube());
			SmartDashboard.putData("Open Arms", new OpenArm());
			SmartDashboard.putData("Spin In", new SuckInCube());
			SmartDashboard.putData("Spin Out", new ThrowCube());

			CommandGroup collect = new CommandGroup("Collect");
			collect.addParallel(new OpenArm());
			collect.addParallel(new SuckInCube());
			SmartDashboard.putData("Collect", collect);
		}

		// Catcher prototype commands (not always connected to robot)
		if (CollectorPrototypeSubsystem.isConnected()) {
			SmartDashboard.putData("Open Catcher", new OpenArm());
			SmartDashboard.putData("Close Catcher", new CloseArm());
			SmartDashboard.putData("Push Catchar", new PushCube());
		}
	}

	public static double checkDeadZone(double raw) {
		double mag = Math.abs(raw);
		if (mag < kDeadZone) {
			return 0.0;
		}
		// Take out dead zone hole and return value in range of [-1.0, +1.0]
		if (raw < 0) {
			raw += kDeadZone;
			raw /= (1.0 - kDeadZone);
		} else {
			raw -= kDeadZone;
			raw /= (1.0 - kDeadZone);
		}
		return raw;
	}

	/**
	 * Get throttle for driving in arcade mode.
	 * 
	 * @return Throttle value based on stick Y axis (value in range of [-1.0, +1.0].
	 */
	public double getThrottle() {
		double axis = m_driver.getRawAxis(5);
		return checkDeadZone(-axis);
	}

	/**
	 * Get turn value for driving in arcade mode.
	 * 
	 * @return How much to turn based on stick X axis (value in range of [-1.0,
	 *         +1.0].
	 */
	public double getSteer() {
		double axis = m_driver.getRawAxis(2);
		return checkDeadZone(axis);
	}

	public double getTankRight() {
		double rightStick = -m_driver.getRawAxis(5);
		rightStick *= 5;
		if (rightStick < 0) {
			rightStick = Math.ceil(rightStick);
		} else {
			rightStick = Math.floor(rightStick);
		}
		rightStick /= 5;
		return rightStick;
	}

	public double getTankLeft() {
		double leftStick = -m_driver.getRawAxis(1);
		leftStick *= 5;
		leftStick = Math.floor(leftStick);
		leftStick /= 5;
		return leftStick;
	}
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
