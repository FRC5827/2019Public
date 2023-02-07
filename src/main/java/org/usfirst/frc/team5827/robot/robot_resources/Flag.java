package org.usfirst.frc.team5827.robot.robot_resources;

import org.usfirst.frc.team5827.robot.Logging;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flag
{
    private	Servo servo;
    	
	public Flag() {
		servo = new Servo(1);
	}

	public void raise()
	{
        servo.set(0);
        Logging.consoleLog("Servo up");     
	}

	public void lower()
	{
        servo.set(0.9);
        Logging.consoleLog("Servo down");
    }
}
