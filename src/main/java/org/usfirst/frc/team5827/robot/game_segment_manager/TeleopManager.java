package org.usfirst.frc.team5827.robot.game_segment_manager;

/**
 * A class to manage teleop, like AutonomousManager.java.
 * 
 * Goal:
 *  - To simplify Robot.java.
 */

 import org.usfirst.frc.team5827.robot.robot_resources.RobotControl;
import org.usfirst.frc.team5827.robot.robot_resources.Shifter;

import edu.wpi.first.wpilibj.Servo;

import org.usfirst.frc.team5827.robot.robot_resources.HatchPanelManipulator;
import org.usfirst.frc.team5827.robot.robot_resources.Climber;
import org.usfirst.frc.team5827.robot.robot_resources.Drive;
import org.usfirst.frc.team5827.robot.robot_resources.Flag;
import org.usfirst.frc.team5827.robot.robot_resources.OperatorControl;
// import org.usfirst.frc.team5827.robot.autonomous.AutonomousPatterns;
//import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;
import org.usfirst.frc.team5827.robot.Logging;

 public class TeleopManager extends GameSegmentManager
 {
    private boolean inAutonomousPattern = false;
    private boolean turnauton = false;
    // private AutonomousManager autoManager;
    private Servo servo;

    Drive drive;
    HatchPanelManipulator hatch;
    OperatorControl operator;
    Shifter shifter;
    Climber climb;
    Flag flag;

    // The given AutonomousManager allows autonomous patterns to be run in
    //teleop.
    public TeleopManager(RobotControl robotResources) //, AutonomousManager autonomousManager)
    {
        super(robotResources);

        drive = robotResourceManager.getDrive();
        hatch = robotResourceManager.getHatchManipulator();
        operator = robotResourceManager.getOperatorControl();
        shifter = robotResourceManager.getShifter();
        climb = robotResourceManager.getClimber();
        flag = robotResourceManager.getFlag();

    }

    // On the initialization of teleop.
    @Override
    public void onModeInit()
    {
        drive.setDriveMode(Drive.DriveMode.CURVATURE_DRIVE);
        drive.setBrakeMode(true);
        climb.setBrakeMode(true);
        climb.stop();
        drive.simpleArcadeDrive(0, 0);
    }

    // Called periodically during teleop.
    @Override
    public void periodic()
    {
        //Drive using joystick input
        double forwardAmount = -(operator.getAxisAmount("Right Trigger") - operator.getAxisAmount("Left Trigger")),
                turnAmount = operator.getTurn();
        
        Logging.consoleLog("Forward: %.2f, Turn: %.2f", forwardAmount, turnAmount);

		// Check for button presses and activate automated functions
        
        if (operator.getPOV() == 180) {
            if(operator.getButtonIsPressed("Button Y")) {
                flag.raise();
            }
            else if(operator.getButtonIsPressed("Button A")) {
                flag.lower();
            }
        }
        else if (operator.getPOV() == 90){
            hatch.extend();
        }
        else if (operator.getPOV() == 270){
            hatch.retract();
        }
        else if (operator.getButtonIsPressed("Button B"))
        {
            hatch.outtake();
        }
        else if (operator.getButtonIsPressed("Button A"))
        {
            hatch.intake();
        }
        else if (operator.getButtonIsPressed("Right Bumper"))
        {
			shifter.highGear();
        }
        else if (operator.getButtonIsPressed("Left Bumper"))
        {
			shifter.lowGear();
        }

        if(Math.abs(turnAmount) < 0.05) {
            turnAmount = 0;
        }
        drive.drive(forwardAmount, turnAmount);
    }

    @Override
    public void onModeDisable()
    {

    }
 }