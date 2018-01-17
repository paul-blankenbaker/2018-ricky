package org.usfirst.frc.team868.robot.subsystems;

import org.usfirst.frc.team868.robot.Robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CollectorSubsystem extends Subsystem {
	
	private static final int kLeftPWM = 5;
	private static final int kLeftPDP = 4;
	private static final int kRightPWM = 6;
	private static final int kRightPDP = 3;
	private static final boolean kDebug = true;
	private Solenoid catchBar;
	private Spark spinLeft;
	private Spark spinRight;
	private double maxSpinCurrent;

	public CollectorSubsystem() {
		catchBar = new Solenoid(5);

		spinLeft = new Spark(kLeftPWM);
		spinRight = new Spark(kRightPWM);
		
		double spinPower = SmartDashboard.getNumber("SpinPower", 0.5);
		SmartDashboard.putNumber("SpinPower", spinPower);
	}
	
	public void open() {
		catchBar.set(false);
	}
	
	public void close() {
		catchBar.set(true);
	}
	
	private double getSpinPower() {
		double spinPower = SmartDashboard.getNumber("SpinPower", 0.5);
		return spinPower;
	}

	public void spinIn() {
		double spinPower = getSpinPower();
		spinLeft.set(spinPower);
		spinRight.set(-spinPower);
	}

	public void spinOut() {
		double spinPower = getSpinPower();
		spinLeft.set(-spinPower);
		spinRight.set(spinPower);
	}
	
	public double getSpinCurrent() {
		double leftCurrent = Math.abs(Robot.kPDP.getCurrent(kLeftPDP));
		double rightCurrent = Math.abs(Robot.kPDP.getCurrent(kRightPDP));
		double totalCurrent = Math.max(leftCurrent, rightCurrent);
		maxSpinCurrent = Math.max(maxSpinCurrent, totalCurrent);
		return totalCurrent;
	}
	
	public void updateDashboard() {
		if (kDebug) {
			getSpinCurrent();
			SmartDashboard.putNumber("Max Spin Current", maxSpinCurrent);
		}
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	public void spinStop() {
		spinLeft.set(0);
		spinRight.set(0);
	}
}

