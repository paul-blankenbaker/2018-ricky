package org.usfirst.frc.team868.robot.subsystems;

import org.usfirst.frc.team868.robot.Robot;
import org.usfirst.frc.team868.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Temporary prototype collector with spinning wheels on two arms that can squeeze in to
 * collect cubes.
 */
public final class CollectorPrototypeSubsystem extends Subsystem {

	private static final int kLeftPWM = 5;
	private static final int kLeftPDP = 4;
	private static final int kRightPWM = 6;
	private static final int kRightPDP = 3;
	private static final boolean kDebug = true;
	private Solenoid catchBar;
	private SpeedController spinLeft;
	private SpeedController spinRight;
	private double maxSpinCurrent;
	
	/**
	 * NOTE: If you set this to true, make sure you set Catcher's flag to false - only one can be connected!
	 */
	private static final boolean isConnected = false;

	/**
	 * Construct a new instance of the prototype collector.
	 * 
	 * @param isConnected
	 *            Pass true if prototype collector has been connected to the
	 *            subsystem.
	 */
	public CollectorPrototypeSubsystem() {

		if (isConnected) {
			catchBar = new Solenoid(RobotMap.PCM.kCollectorGrabber);

			spinLeft = new Spark(kLeftPWM);
			spinRight = new Spark(kRightPWM);
		}

		double spinPower = SmartDashboard.getNumber("SpinPower", 0.5);
		SmartDashboard.putNumber("SpinPower", spinPower);
	}

	/**
	 * Determine if collector prototype is connected to the robot.
	 * 
	 * @return true if collector is connected, false if all operations are no-ops.
	 */
	public static boolean isConnected() {
		return isConnected;
	}

	public void open() {
		if (isConnected) {
			catchBar.set(false);
		}
	}

	public void close() {
		if (isConnected) {
			catchBar.set(true);
		}
	}

	private double getSpinPower() {
		double spinPower = SmartDashboard.getNumber("SpinPower", 0.5);
		return spinPower;
	}

	public void spinIn() {
		double spinPower = getSpinPower();
		if (isConnected) {
			spinLeft.set(spinPower);
			spinRight.set(-spinPower);
		}
	}

	public void spinOut() {
		double spinPower = getSpinPower();
		if (isConnected) {
			spinLeft.set(-spinPower);
			spinRight.set(spinPower);
		}
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
		// setDefaultCommand(new MySpecialCommand());
	}

	public void spinStop() {
		if (isConnected) {
			spinLeft.set(0);
			spinRight.set(0);
		}
	}
}
