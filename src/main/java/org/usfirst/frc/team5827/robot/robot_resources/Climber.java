package org.usfirst.frc.team5827.robot.robot_resources;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team5827.robot.Logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber
{
    private	WPI_TalonSRX climbMotorL;
    private WPI_TalonSRX climbMotorR;
	
	public Climber() {
        final int TIMEOUT = 30;
        ErrorCode errorCodeResult = ErrorCode.OK;

		climbMotorL = new WPI_TalonSRX(RobotMap.MotorControllerPorts.LeftClimb);
        climbMotorR = new WPI_TalonSRX(RobotMap.MotorControllerPorts.RightClimb);
       
        errorCodeResult = climbMotorR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, TIMEOUT);
        if (errorCodeResult != ErrorCode.OK) {
            Logging.consoleLog("configSelectedFeedbackSensor() failed, errorcode " + errorCodeResult.toString());
        }

        errorCodeResult = climbMotorR.configFeedbackNotContinuous(true, TIMEOUT); 
        if (errorCodeResult != ErrorCode.OK)  {
            Logging.consoleLog("configFeedbackNotContinuous() failed, errorcode " + errorCodeResult.toString());
        }
        
        //MotionMagicConfig();
        climbMotorL.follow(climbMotorR);
	}

	public void climb(double p)
	{
        // and with 0xFFF to keep lower 12 bits.  Mod in java (%) has negative result if dividend is negative.
        // Value of 1000 will need to be changed if climber design is modified, and is robot specific based on encoder magnet position
        int currentEncoderPosition = ((climbMotorR.getSensorCollection().getPulseWidthPosition() - (RobotMap.Constants.encoderMin - 1000)) & 0xFFF) + (RobotMap.Constants.encoderMin - 1000);
        if ((currentEncoderPosition > RobotMap.Constants.encoderMax && p > 0) ||
            (currentEncoderPosition < RobotMap.Constants.encoderMin && p < 0))
        {
            p *= 0.1;
        }
        climbMotorR.set(p);
        SmartDashboard.putNumber("zzz Lift Encoder", currentEncoderPosition);
        //Logging.consoleLog("LiftEncoder %f",(double)(climbMotorR.getSensorCollection().getPulseWidthPosition()+2048)%4096-2048);  
	}

	public void stop()
	{
		climb(0.0);
    }
    
    public void setBrakeMode(boolean toBrake) {

        if(toBrake) {
            climbMotorL.setNeutralMode(NeutralMode.Brake);
            climbMotorR.setNeutralMode(NeutralMode.Brake);
        }
        else {
            climbMotorL.setNeutralMode(NeutralMode.Coast);
            climbMotorR.setNeutralMode(NeutralMode.Coast);
        }
    }
    public int getEncoderValue() {
        return (climbMotorR.getSensorCollection().getPulseWidthPosition() - RobotMap.Constants.encoderMin) & 0xFFF;
    }
    public void resetEncoder(){
    }

    public void ramp(double seconds) {
        climbMotorR.configOpenloopRamp(seconds);
    }
}
