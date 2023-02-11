package org.usfirst.frc.team5827.robot;
//import edu.wpi.first.wpilibj.PIDSource;
//import edu.wpi.first.wpilibj.PIDSourceType;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
/**
 * Rotation Based Encoder
 * @author Efraim 
 *
 */
public class RotEncoder //implements PIDSource
{
	
	public boolean isInverted = false;
	
	//private PIDSourceType sourcetype;
	private SensorCollection sensor;
	
	public RotEncoder(SensorCollection sensor) {
		//sourcetype = PIDSourceType.kDisplacement;
		this.sensor = sensor;
	}
	
	public void reset() {
		ErrorCode error = sensor.setQuadraturePosition(0, 5);
		//ErrorCode error = RobotMap.liftMotor2.setSelectedSensorPosition(0, 0, 5);
		//if (error != null) {
		//	Logging.consoleLog(error.toString());
		//}
	}
	
	// Get the value of the encoder
	public int getValue() {
		return sensor.getQuadraturePosition();
		//return RobotMap.liftMotor2.getSelectedSensorPosition(0);
	}
	
	// @Override
	// public void setPIDSourceType(PIDSourceType pidSource)
	// {
	// 	//sourcetype = pidSource;
		
	// }

	// @Override
	// public PIDSourceType getPIDSourceType()
	// {
	// 	return sourcetype;
	// }

	//@Override
	public double pidGet()
	{
		if(isInverted)
			return (-1*sensor.getQuadraturePosition());
		return sensor.getQuadraturePosition();
	}
	

}
