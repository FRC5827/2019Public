// package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

// /**
//  * An autonomous pattern to point the robot at a target visible
//  * to the limelight.
//  */

// import org.usfirst.frc.team5827.robot.robot_resources.Drive;
// import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;

//  public class TurnToTarget extends AutonCommand
//  {
//      protected double m_minimumAngle;

//      public TurnToTarget(double duration, double minimumAngle)
//      {
//         super(duration);

//         m_minimumAngle = minimumAngle;
//      }

//      // Start the TurnToTarget drive.
//      @Override
//      protected void onCommandStart()
//      {
//         Drive drive = m_robotControl.getDrive();
//         drive.pointAtTargetDrive();
//      }

//      // Get whether it should be stopped.
//      @Override
//      public boolean getShouldStop()
//      {
//         boolean shouldStop = m_minimumAngle > Math.abs(LimeLightConnector.getXOffset())
//              && LimeLightConnector.getTargetArea() > 0.01; // And the limelight has the target

//         return shouldStop;
//      }
//  }