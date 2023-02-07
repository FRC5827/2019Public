package org.usfirst.frc.team5827.robot.robot_resources;

import org.usfirst.frc.team5827.robot.Logging;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Shifter
 * @author Matthew
 */
public class Shifter
{
    private DoubleSolenoid shifter = null;

	public Shifter() {
		if (RobotMap.SolenoidOptions.USE_SOLENOIDS) 
		{
			shifter = new DoubleSolenoid(RobotMap.SolenoidOptions.SHIFTER_ID,
				RobotMap.SolenoidOptions.SHIFTER_FORWARD_CHANNEL, RobotMap.SolenoidOptions.SHIFTER_BACKWARD_CHANNEL);
		}
	}

	/**
	 * Shift to low gear
	 */
	public void lowGear() {
		if (RobotMap.SolenoidOptions.USE_SOLENOIDS) 
		{
            	shifter.set(DoubleSolenoid.Value.kForward);
            	Logging.consoleLog("Shift to low gear");
		}
	}

	/**
	 * Shift to high gear
	 */
	public void highGear() {
		if (RobotMap.SolenoidOptions.USE_SOLENOIDS)
		{
            	shifter.set(DoubleSolenoid.Value.kReverse);
            	Logging.consoleLog("Shift to high gear");
		}
    }
}