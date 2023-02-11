// package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

// /**
//  * A command to drive the robot a specific distance.
//  */

// public class DriveDistance extends AutonCommand
// {
//     protected double distanceToDrive;

//     // Initialize with a maximum duration and distance to travel.
//     public DriveDistance(double duration, double distance)
//     {
//         super(duration);

//         distanceToDrive = distance;
//     }

//     // Start the distance drive.
//     @Override
//     protected void onCommandStart()
//     {
//         m_robotControl.getDrive().distanceDrive(distanceToDrive);
//     }

//     //uncomment for motion magic
//     /*
//     @Override
//     public void commandPeriodic() 
//     {
//         //do nothing (overrides arcade drive)
//     }
//     */
// }