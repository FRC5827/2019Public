package org.usfirst.frc.team5827.robot.robot_resources;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorControl{
	private Joystick joystick;
	private HashMap<String, Integer> buttonPorts = new HashMap<String, Integer>();
	private HashMap<String, Integer> axisPorts = new HashMap<String, Integer>();

	public OperatorControl(){ 
        joystick = new Joystick(RobotMap.Joysticks.Joystick);

        buttonPorts.put("Button A", 1);
        buttonPorts.put("Button B", 2);
        buttonPorts.put("Button X", 3);
        buttonPorts.put("Button Y", 4);
        buttonPorts.put("Left Bumper", 5);
        buttonPorts.put("Right Bumper", 6);
        buttonPorts.put("Left Stick Button", 9);
        buttonPorts.put("Right Stick Button", 10);
        buttonPorts.put("Start Button", 8);
        buttonPorts.put("Back Button", 7);
            
        axisPorts.put("HZ Left Stick", 0);
        axisPorts.put("VT Left Stick", 1);
        axisPorts.put("HZ Right Stick", 4);
        axisPorts.put("VT Right Stick", 5);
        axisPorts.put("Left Trigger", 2);
        axisPorts.put("Right Trigger", 3);
    }
    public double getAxisAmount(String axisName){
		Integer axisPort = axisPorts.get(axisName);

		return joystick.getRawAxis(axisPort.intValue());
    }
    public boolean getButtonIsPressed(String buttonName){     
        Integer buttonPort = buttonPorts.get(buttonName);

		int buttonID = buttonPort.intValue();

        return joystick.getRawButton(buttonID);
    }
    public double getTurn(){
		double pow = getAxisAmount("HZ Left Stick");
		return pow;
    }
    public boolean shiftHighGear(){
		return getButtonIsPressed("Left Bumper");
    }
    public boolean shiftLowGear(){
		return getButtonIsPressed("Right Bumper");
    }
    public int getPOV() {
		return joystick.getPOV();
	}
}
