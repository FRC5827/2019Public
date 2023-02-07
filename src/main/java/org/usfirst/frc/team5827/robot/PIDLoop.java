package org.usfirst.frc.team5827.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Versatile Class for PID Loops
 * @author Matthew
 */
public class PIDLoop implements PIDOutput
{
	//base FRC PIDController
	private PIDController PIDController;

	//the output of the PID loop. Can be accessed using the getPIDOutput method
	private double pidOutput;

	/**
	 * Create a new PID Loop (use this with limitless input values, such as encoders)
	 * @param name Name of PID loop for shuffleboard
	 * @param Kp Proportional constant
	 * @param Ki Integral constant
	 * @param Kd Derivative constant
	 * @param source Input source
	 * @param kTolerance Acceptable error
     * @param continuous Whether the PIDSource's output is a continuous range
     * (such as a -180 to 180 angle range, in which 180 and -180 are the same point, so that shifting from -179.99 to 179.99
     * is not a problem.)
	 */
	public PIDLoop(String name, double Kp, double Ki, double Kd, PIDSource source, double kTolerance) {
		PIDController = new PIDController(Kp, Ki, Kd, source, this, .02);
		PIDController.setAbsoluteTolerance(kTolerance);
		PIDController.reset();
		PIDController.setName(name);
		SmartDashboard.putData(name, PIDController);
    }
    
	/**
	 * Create a new PID Loop (use this with limited input values, such as gyro values)
	 * @param name Name of PID loop for shuffleboard
	 * @param Kp Proportional constant
	 * @param Ki Integral constant
	 * @param Kd Derivative constant
	 * @param source Input source
	 * @param kTolerance Acceptable error
	 * @param inputMin Minimum input value
	 * @param inputMax Maximum input value
	 */
	public PIDLoop(String name, double Kp, double Ki, double Kd, PIDSource source, double inputMin, double inputMax, double kTolerance) {
		this(name, Kp, Ki, Kd, source, inputMin, inputMax, kTolerance, true); // Default to continuous input.
    }
    
    /**
	 * Create a new PID Loop (use this with limited input values, such as gyro values)
	 * @param name Name of PID loop for shuffleboard
	 * @param Kp Proportional constant
	 * @param Ki Integral constant
	 * @param Kd Derivative constant
	 * @param source Input source
	 * @param kTolerance Acceptable error
	 * @param inputMin Minimum input value
	 * @param inputMax Maximum input value
     * @param continuous Whether the PIDSource's output is a continuous range
     * (such as a -180 to 180 angle range, in which 180 and -180 are the same point, so that shifting from -179.99 to 179.99
     * is not a problem.)
	 */
	public PIDLoop(String name, double Kp, double Ki, double Kd, PIDSource source, double inputMin, double inputMax, double kTolerance, boolean continuous) {
		PIDController = new PIDController(Kp, Ki, Kd, source, this, .02);
		PIDController.setInputRange(inputMin, inputMax);
		PIDController.setAbsoluteTolerance(kTolerance);
		PIDController.setContinuous(continuous);
		PIDController.reset();
		PIDController.setName(name);
		SmartDashboard.putData(name, PIDController);
	}
	

	//reset the PIDController and disable it
	public void reset() {
        PIDController.reset();
        pidOutput = 0.0;
    }
    
    //if enabled, reset the PIDController and disable it
    public void resetIfEnabled() {
        if(PIDController.isEnabled()) {
            reset();
        }
    }

	//enable the PIDController
	public void enable() {
		PIDController.enable();
	}

	//disable the PIDController
	public void disable() {
        PIDController.disable();
        pidOutput = 0.0;
	}

	/**
	 * sets a new setpoint for the PIDController
	 * @param setpoint value that the PID loop will approach
	 */
	public void setSetpoint(double setpoint) {
		PIDController.setSetpoint(setpoint);
	}

	/**
	 * Gets the setpoint of the PIDController.
	 * @return The value the PID loop is approaching.
	 */
	public double getSetpoint()
	{
		return PIDController.getSetpoint();
	}

	/**
	 * returns the PID loop's output
	 * @return the rate at which the input (angle, position, etc.) should be modified
	 * example: in a speed controller PID loop using encoders, this would be the speed value
	 */
	public double getPIDOutput() {
		return pidOutput;
	}

	@Override
	/**
	 * This function is invoked periodically by the PIDController
	 * @param output output from the PIDController
	 */
	public void pidWrite(double output)
	{
		pidOutput = output;
	}
}