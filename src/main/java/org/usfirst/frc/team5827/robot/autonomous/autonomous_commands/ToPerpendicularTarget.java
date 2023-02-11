// package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

// /**
//  * Drive the robot to a target that is already perpindicular to the robot.
//  */

// import org.usfirst.frc.team5827.robot.robot_resources.Drive;
// //import org.usfirst.frc.team5827.robot.robot_resources.RobotMap;
// //import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;

// public class ToPerpendicularTarget extends AutonCommand
// {
//     private double distanceToGetTo;

//     // Initialize with a maximum duration and a distance to get to the target.
//     public ToPerpendicularTarget(double duration, double distanceToAttain)
//     {
//         super(duration);

//         distanceToGetTo = distanceToAttain;
//     }

//     // Start the command.
//     @Override
//     protected void onCommandStart()
//     {
//         Drive drive = m_robotControl.getDrive();
//         drive.distanceToTargetDrive(distanceToGetTo / 12.0); // Convert to feet.
//     }

//     //Get whether the command should be stopped.
//     @Override
//     public boolean getShouldStop()
//     {
//         double distance = LimeLightConnector.getDistance(),
//             distanceRemaining = Math.abs(distance - distanceToGetTo);

//         boolean hasTarget = LimeLightConnector.hasTarget();

//         boolean shouldStop = distanceRemaining <= RobotMap.Constants.limelightDistanceTolerance || !hasTarget;

//         return shouldStop;
//     }
// }