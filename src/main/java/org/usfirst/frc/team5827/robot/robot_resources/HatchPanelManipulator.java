package org.usfirst.frc.team5827.robot.robot_resources;

import org.usfirst.frc.team5827.robot.Logging;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Hatch Panel Manipulator
 * @author Matthew
 */
public class HatchPanelManipulator
{
	private static final boolean USE_SOLENOID = true;

    private DoubleSolenoid hatch = null;

    private DoubleSolenoid extender = null;

    private boolean extended;

	public HatchPanelManipulator() {
		if (USE_SOLENOID) {
						// Initialize solenoids from RobotMap ids.
						hatch = new DoubleSolenoid(RobotMap.SolenoidOptions.HATCH_ID, 
																			 RobotMap.SolenoidOptions.HATCH_FORWARD_CHANNEL,
																			 RobotMap.SolenoidOptions.HATCH_BACKWARD_CHANNEL);
						extender = new DoubleSolenoid(RobotMap.SolenoidOptions.EXTENDER_ID, 
																					RobotMap.SolenoidOptions.EXTENDER_FORWARD_CHANNEL,
                                                                                    RobotMap.SolenoidOptions.EXTENDER_BACKWARD_CHANNEL);
                        extended = false;
		}
	}

	/**
	 * Grab on to a hatch panel
	 */
	public void intake() {
		if (USE_SOLENOID) {
			hatch.set(DoubleSolenoid.Value.kReverse);
			Logging.consoleLog("Intake");
		}
	}

	/**
	 * Release a hatch panel
	 */
	public void outtake() {
		if (USE_SOLENOID) {
			hatch.set(DoubleSolenoid.Value.kForward);
			Logging.consoleLog("Outtake");
		}
    }
    
    /**
	 * Extend the hatch panel manipulator
	 */
	public void extend() {
		if (USE_SOLENOID) {
			extender.set(DoubleSolenoid.Value.kForward);
            Logging.consoleLog("Extend");
            extended = true;
		}
    }

    /**
	 * Retract the hatch panel manipulator
	 */
	public void retract() {
		if (USE_SOLENOID) {
			extender.set(DoubleSolenoid.Value.kReverse);
            Logging.consoleLog("Retract");
            extended = false;
		}
    }

    public void toggleExtender() {
        if(extended) {
            retract();
        }
        else {
            extend();
        }
    }
}