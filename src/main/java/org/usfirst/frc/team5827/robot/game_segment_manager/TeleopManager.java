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
 import org.usfirst.frc.team5827.robot.autonomous.AutonomousPatterns;
import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;
import org.usfirst.frc.team5827.robot.Logging;

 public class TeleopManager extends GameSegmentManager
 {
    private boolean inAutonomousPattern = false;
    private boolean turnauton = false;
    private AutonomousManager autoManager;
    private Servo servo;

    Drive drive;
    HatchPanelManipulator hatch;
    OperatorControl operator;
    Shifter shifter;
    Climber climb;
    Flag flag;

    // The given AutonomousManager allows autonomous patterns to be run in
    //teleop.
    public TeleopManager(RobotControl robotResources, AutonomousManager autonomousManager)
    {
        super(robotResources);

        autoManager = autonomousManager;

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

        //Extend Hatch Manipulator
        hatch.extend();

        drive.setDriveMode(Drive.DriveMode.CURVATURE_DRIVE);
        drive.setBrakeMode(true);
        climb.setBrakeMode(true);
        climb.stop();
        drive.simpleArcadeDrive(0, 0);
        autoManager.onModeDisable();

        flag.raise();
    }

    // Called periodically during teleop.
    @Override
    public void periodic()
    {
        //Drive using joystick input
        double forwardAmount = -(operator.getAxisAmount("Right Trigger") - operator.getAxisAmount("Left Trigger")),
                turnAmount = operator.getTurn();
        
        //Logging.consoleLog("Forward: %.2f, Turn: %.2f", forwardAmount, turnAmount);

		//Check for button presses and activate automated functions
        
        if (operator.getPOV() == 180) {
            if(operator.getButtonIsPressed("Button Y")) {
                flag.raise();
            }
            else if(operator.getButtonIsPressed("Button A")) {
                flag.lower();
            }
            else if(operator.getButtonIsPressed("Button X")) {
                inAutonomousPattern = true;

                // Initialize autonomous.
                autoManager.configureDrive();

                // Go to the target.
                autoManager.selectPattern(AutonomousPatterns.Pattern.LOWER_LEFT);

                // Log this.
                Logging.consoleLog("Started Autonomous.");
            }
            else if(operator.getButtonIsPressed("Button B")) {
                inAutonomousPattern = true;

                // Initialize autonomous.
                autoManager.configureDrive();

                // Go to the target.
                autoManager.selectPattern(AutonomousPatterns.Pattern.LOWER_RIGHT);

                // Log this.
                Logging.consoleLog("Started Autonomous.");
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
        else if (operator.getButtonIsPressed("Button X"))
        {
			inAutonomousPattern = true;

            LimeLightConnector.setLedMode(true);
            LimeLightConnector.setSnapshot(true);

            // Initialize autonomous.
            autoManager.configureDrive();

            // Go to the target.
            autoManager.selectPattern(AutonomousPatterns.Pattern.GET_HATCH);

            // Log this.
            Logging.consoleLog("Started Autonomous.");
		}
        else if (operator.getButtonIsPressed("Button A"))
        {
            hatch.intake();
        }
        else if (operator.getButtonIsPressed("Button Y"))
        {
            inAutonomousPattern = true;

            LimeLightConnector.setLedMode(true);
            LimeLightConnector.setSnapshot(true);

            // Initialize autonomous.
            autoManager.configureDrive();

            // Go to the target.
            autoManager.selectPattern(AutonomousPatterns.Pattern.PLACE_HATCH);

            // Log this.
            Logging.consoleLog("Started Autonomous.");
        }
        else if (operator.getButtonIsPressed("Start Button"))
        {
            turnauton = true;

            // Initialize autonomous.
            autoManager.configureDrive();

            // Log this.
            Logging.consoleLog("Started Autonomous.");

			drive.AutoTurn(90);
        }
        else if (operator.getButtonIsPressed("Back Button"))
        {
            turnauton = true;

            // Initialize autonomous.
            autoManager.configureDrive();

            // Log this.
            Logging.consoleLog("Started Autonomous.");

			drive.AutoTurn(-90);
        }
        else if (operator.getButtonIsPressed("Right Bumper"))
        {
			shifter.highGear();
        }
        else if (operator.getButtonIsPressed("Left Bumper"))
        {
			shifter.lowGear();
        }
        else if (operator.getPOV() == 0) {
            inAutonomousPattern = true;

            // Initialize autonomous.
            autoManager.configureDrive();

            // Go to the target.
            autoManager.selectPattern(AutonomousPatterns.Pattern.CLIMB);

            // Log this.
            Logging.consoleLog("Started Autonomous.");
        }

        if(Math.abs(operator.getAxisAmount("VT Right Stick")) > 0.05) {
            climb.climb(operator.getAxisAmount("VT Right Stick") * -0.8);
        }
        else if (!inAutonomousPattern){
            climb.stop();
        }

        // If moving/turning,
        if ((Math.abs(forwardAmount) > 0.1 || Math.abs(turnAmount) > 0.1)
                && (inAutonomousPattern || turnauton))
        {
            inAutonomousPattern = false;

            turnauton = false;

            // Stop the current command.
            autoManager.onModeDisable();
            drive.stop();
            LimeLightConnector.setSnapshot(false);

            // Go back to curvature drive
            drive.setDriveMode(Drive.DriveMode.CURVATURE_DRIVE);

            //shifter.highGear();

            // Log this.
            Logging.consoleLog("Stopped autonomous.");
        }

        // If in an autonomous pattern, call the autonomous manager.
        if (inAutonomousPattern)
        {
            autoManager.periodic();
        }
        else
        {
            if(Math.abs(turnAmount) < 0.05) {
                turnAmount = 0;
            }
            drive.drive(forwardAmount, turnAmount);
        }
    }

    @Override
    public void onModeDisable()
    {

    }
 }