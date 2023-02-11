/*
 * FRC Destination Deep Space 2019 Robot Code for Team 5827
 */

package org.usfirst.frc.team5827.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;

import org.usfirst.frc.team5827.robot.dashboard_manager.SmartDashboardManager;

//import org.usfirst.frc.team5827.robot.game_segment_manager.AutonomousManager;
import org.usfirst.frc.team5827.robot.game_segment_manager.GameSegmentManager;
import org.usfirst.frc.team5827.robot.game_segment_manager.TeleopManager;
//import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;
import org.usfirst.frc.team5827.robot.robot_resources.Climber;
import org.usfirst.frc.team5827.robot.robot_resources.Drive;
import org.usfirst.frc.team5827.robot.robot_resources.HatchPanelManipulator;
import org.usfirst.frc.team5827.robot.robot_resources.OperatorControl;
import org.usfirst.frc.team5827.robot.robot_resources.RobotControl;

public class Robot extends TimedRobot
{
    // Resources
    protected RobotControl robotResources;

    //Smart Dashboard
    protected SmartDashboardManager dashboardManager;

    // Teleop and autonomous managers.
    //protected AutonomousManager autoManager;
    protected TeleopManager teleopManager;
    protected GameSegmentManager currentSegment = null;

	@Override
	public void robotInit()
	{

        // Initialize the limelight.
        // LimeLightConnector.initializeIfNeeded();
        // LimeLightConnector.setCameraMode(true);

		//initialize drive base and operator control
        robotResources = new RobotControl();
        dashboardManager = SmartDashboardManager.getInstance();

        // Initialize the autonomous and teleop managers.
        //autoManager = new AutonomousManager(robotResources);
        teleopManager = new TeleopManager(robotResources, autoManager);

        // Start the camera server
        CameraServer.startAutomaticCapture();

        Climber climb = robotResources.getClimber();

	}

	//@Override
	// public void autonomousInit()
	// {
    //     autoManager.onModeInit();

    //     currentSegment = autoManager;
	// }

	// @Override
	// public void autonomousPeriodic()
	// {
    //     teleopManager.periodic();

    //     // Update data on the Smart Dashboard.
    //     dashboardManager.update();
    // }

	@Override
	public void teleopInit()
	{
        teleopManager.onModeInit();
        
        currentSegment = teleopManager;
	}

	@Override
	public void teleopPeriodic()
	{
		teleopManager.periodic();

        // Update data on the Smart Dashboard.
        dashboardManager.update();
	}

	@Override
	public void testInit()
	{
	}

	@Override
	public void testPeriodic()
	{
        // Retrieve resources.
        Drive drive = robotResources.getDrive();
        HatchPanelManipulator hatch = robotResources.getHatchManipulator();
        OperatorControl operator = robotResources.getOperatorControl();
        dashboardManager.update();
        if (operator.getButtonIsPressed("Button B"))
        {
			drive.setRight1();
        }
        else if (operator.getButtonIsPressed("Button X"))
        {
			drive.setRight5();
		}
        else if (operator.getButtonIsPressed("Button A"))
        {
			drive.setLeft4();
        }
        else if (operator.getButtonIsPressed("Button Y"))
        {
            drive.setLeft2();
        }
        else {
            drive.stopMotors();
        }
        /*if(operator.getButtonIsPressed("Right Bumper")){
            drive.MotionMagicFeet(10);
        }
        Logging.consoleLog("encoder ticks: %f",(double)drive.leftTalon.getSelectedSensorPosition());*/
    }
    
    @Override
    public void disabledInit()
    {
        if(currentSegment != null)
        {
            currentSegment.onModeDisable();
        }

        //autoManager.onModeDisable();

        Drive drive = robotResources.getDrive();
        drive.stop();
        drive.setBrakeMode(false);
        Climber climb = robotResources.getClimber();
        //climb.setBrakeMode(false);
    }
}
