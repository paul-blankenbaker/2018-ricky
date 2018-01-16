package org.usfirst.frc.team868.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;


/**
 *
 */
public class Catcher extends Subsystem {
	private Solenoid catchBar;
	private Solenoid puncher;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Catcher() {
		catchBar = new Solenoid(5);
		puncher = new Solenoid(7);
	}
	
	public void open() {
		catchBar.set(false);
		puncher.set(true);
	}
	
	public void close() {
		catchBar.set(true);
		puncher.set(true);
	}
	
	public void push() {
		puncher.set(false);
		catchBar.set(false);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

