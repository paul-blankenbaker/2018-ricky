/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team868.robot;

import org.usfirst.frc.team868.robot.commands.CloseArm;
import org.usfirst.frc.team868.robot.commands.GrabCube;
import org.usfirst.frc.team868.robot.commands.OpenArm;
import org.usfirst.frc.team868.robot.commands.PushCube;
import org.usfirst.frc.team868.robot.commands.SuckInCube;
import org.usfirst.frc.team868.robot.commands.ThrowCube;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private Joystick stick;

	public OI() {
		stick = new Joystick(0);
		
		setupSmartDashboard();
	}
	
	private void setupSmartDashboard() {
		// IntakeOutput commands
		SmartDashboard.putData("Close Arms", new GrabCube());
		SmartDashboard.putData("Open Arms", new OpenArm());
		SmartDashboard.putData("Spin In", new SuckInCube());
		SmartDashboard.putData("Spin Out", new ThrowCube());

		CommandGroup collect = new CommandGroup("Collect");
		collect.addParallel(new OpenArm());
		collect.addParallel(new SuckInCube());
		SmartDashboard.putData("Collect", collect);
		
		// Add Jimmy commands
		SmartDashboard.putData("Open Catcher", new OpenArm());
		SmartDashboard.putData("Close Catcher", new CloseArm());
		SmartDashboard.putData("Push Catchar", new PushCube());
	}

	public double getTankRight() {
		double rightStick = -stick.getRawAxis(5);
		rightStick *= 5;
		if(rightStick < 0) {
			rightStick = Math.ceil(rightStick);
		} else {
			rightStick = Math.floor(rightStick);
		}
		rightStick /= 5;
		return rightStick;
	}
	public double getTankLeft() {
		double leftStick = -stick.getRawAxis(1);
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
