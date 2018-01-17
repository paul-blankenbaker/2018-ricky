package org.usfirst.frc.team868.robot.subsystems;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.util.gyro.BNO055;

/**
 * Subsystem that provides access to information from the gyro.
 * 
 * <p>NOTE: This is a read-only subsystem. Since you can not modify or control it, it is safe to be shared/accessed by multiple commands simultaneously. You do NOT need to require this subsystem in your command's constructor!</p>
 */
public class GyroSubsystem extends Subsystem {

    private final BNO055 gyro;
    private final GyroBase gyroX;
    
    public GyroSubsystem(){
    	gyro = BNO055.getInstance(I2C.Port.kOnboard);
    	gyroX = gyro.createGyroX();
    	gyroX.reset();
    	
		// Assign test mode labeling and group
    	gyroX.setName("Gryo", "Rotation");
    }
    
	/**
	 * Returns a pseudo {@class Gyro} instance that you can use to monitor
	 * rotation about the x-axis (the way the robot is facing).
	 * 
	 * <p>
	 * The Gyro object returned always starts as if "reset" (will be facing
	 * 0 at time of construction). USE THIS IF YOU WANT THE ABILITY TO RESET OR ZERO THE GYRO!
	 * </p>
	 * <p>
	 * NOTE: The {@class Gyro} returned has some limitations/features:
	 * </p>
	 * 
	 * <ul>
	 * <li>The {@link Gyro#calibrate()} method does nothing.</li>
	 * <li>The {@link Gyro#reset()} method only resets that instance to zero
	 * (any other Gyro objects created will be unchanged). This is typically
	 * what you want.</li>
	 * <li>The {@link Gyro#getRate()} method is NOT implemented (do not use it).</li>
	 * </ul>
	 * 
	 * @return A new {@class Gyro} object you can used for tracking rotation.
	 */    
    public GyroBase getRotationGyro() {
    	return gyro.createGyroX();
    }
    
    /**
     * Gets the current rotation of the robot since the program was started.
     * 
     * <p>Only use this if you don't need to zero or reset the gyro!</p>
     * 
     * @return Angle in degrees (no limit - can be less than 0 or larger than 360).
     */
    public double getRotation(){
    	return gyroX.getAngle();
    }
    
    /**
	 * Update information on SmartDashboard.
	 */
    public void updateDashboard() {
    	// Comment out or reduce verbosity unless debugging!
    	gyro.updateDashboard(8);
    	SmartDashboard.putNumber("Rotation", getRotation());
    }

    public void initDefaultCommand() {
    }
}

