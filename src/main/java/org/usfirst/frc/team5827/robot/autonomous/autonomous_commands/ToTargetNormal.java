// package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

// /**
//  * Drive to a target's normal, assuming the angle of the target is measured
//  * at the center of the target, instead of at the target's edge.
//  *  Target
//  *   /
//  *  /.     Drive path (Perpendicular to the normal).
//  * /    . / 
//  *       /   .
//  *      /      Target normal
//  *  -------    (Perpendicular
//  *  |     |     to the target)
//  *  |Robot|                    .
//  *  |     |                          .
//  *  -------
//  * 
//  * TODO Fix getDegreesToTurn in autodrive.
//  */

//  import org.usfirst.frc.team5827.robot.robot_resources.Drive;
//  import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;
//  import org.usfirst.frc.team5827.robot.Logging;

//  import edu.wpi.first.wpilibj.Timer;

//  public class ToTargetNormal extends AutonCommand
//  {
//     // Constants.
//     public static final double FINAL_ANGLE_TO_TARGET_TOLERANCE = 10.0;
//     public static final double ANGLE_TO_NORMAL_TOLERANCE = 24.0,
//                                 ANGLE_TO_NORMAL_MAX_TIME = 1.1, // A maximum duration for the time to turn to the normal.
//                                 DRIVING_TO_TARGET_MAX_TIME = 3.0,
//                                 FINAL_TURN_MAX_TIME = 1.1;

//     // State information.
//     private double initialTurnAngle; // The angle the robot turned initially.
//     private double initialTargetDistance; // The initial distance to the target.
//     private double distanceToDrive;
//     private boolean reRunInit = false;
//     private double startTime;

//     // Pattern state.
//     private enum State { TURNING_TOWARDS_NORMAL, DRIVING_TO_TARGET, TURNING_TOWARDS_TARGET, COMPLETE };
//     private State currentState;

//     // Settings
//     private boolean willPointTowardsTarget = false;

//     // All commands need a maximum duration.
//     public ToTargetNormal(double maximumDuration, boolean pointTowardsTarget)
//     {
//         super(maximumDuration);

//         willPointTowardsTarget = pointTowardsTarget;
//     }

//     @Override
//     public void onCommandStart()
//     {
//         Drive drive = m_robotControl.getDrive();

//         double xOffsetAngle = LimeLightConnector.getXOffset(); // In degrees.
//         double targetAngle = LimeLightConnector.angleOfTarget(); // In degrees.

//         // If an error occured getting the angle to the target,
//         if (targetAngle == LimeLightConnector.ERROR_ANGLE)
//         {
//             // Note that initialization must be re-run.
//             reRunInit = true;

//             return;
//         }

//         // Note that initialization
//         //should have been run successfully.
//         reRunInit = false;

//         double angleToTurn = 90.0 - Math.abs(targetAngle   // A magnitude (we don't want to subtract
//                                  - xOffsetAngle);          // a negative from 90.0 degrees and turn
//                                                            // more than needed/the wrong way). 

//         Logging.consoleLog("Angle to turn: %.2f, targetAngle: %.2f, xOffset: %.2f", angleToTurn, targetAngle, xOffsetAngle);

//         if(targetAngle > 0.0)
//         {
//             angleToTurn *= -1.0; // Turn counterclockwise 
//                                           // if the target is turned counterclockwise (left).
//         }

//         // Store information about the turn.
//         initialTurnAngle = angleToTurn;
//         initialTargetDistance = LimeLightConnector.getDistance();
//         distanceToDrive = initialTargetDistance * Math.cos(initialTurnAngle * Math.PI / 180); // Calculate the distance
//                                                                                               //to drive.

//         // Start driving!
//         drive.AutoTurn(angleToTurn);

//         // Start the timer
//         resetStartTime();

//         // We are now turning to the normal.
//         currentState = State.TURNING_TOWARDS_NORMAL;
//     }

//     // Reset the stored time that a segement of a command was started at.
//     public void resetStartTime()
//     {
//         startTime = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void commandPeriodic()
//     {
//         // If initialization failed,
//         if (reRunInit)
//         {
//             // Try again.
//             onCommandStart();
//         }

//         Drive drive = m_robotControl.getDrive();

//         // Determine the current time and the time since a change
//         //in commands.
//         double nowTime = Timer.getFPGATimestamp();
//         double deltaTime = nowTime - startTime;

//         if(currentState == State.TURNING_TOWARDS_NORMAL)
//         {
//             // Check to see if the turn has completed
//             //(we are within two degrees of the target angle).
//             if (Math.abs(drive.getDegreesToTurn()) < ANGLE_TO_NORMAL_TOLERANCE || deltaTime > ANGLE_TO_NORMAL_MAX_TIME)
//             {
//                 // Stop turning.
//                 drive.stop();

//                 // Drive to the target.
//                 currentState = State.DRIVING_TO_TARGET;

//                 // Drive forwards.
//                 double distanceToDriveFeet = distanceToDrive / 12.0; // DistanceToDrive is in inches.

//                 drive.distanceDrive(distanceToDriveFeet);

//                 // Log information.
//                 Logging.consoleLog("Distance Driving to Target.");

//                 // Get a new timestamp for the command segement start.
//                 resetStartTime();
//             } // End completion check.
//         }
//         else if (currentState == State.DRIVING_TO_TARGET) // Otherwise, if driving towards the normal,
//         {
//             // Check to see if the robot is close enough to the normal
//             //(less than 1/4th of a foot.)
//             if (Math.abs(drive.getDistanceToTravel()) < 0.25 || deltaTime > DRIVING_TO_TARGET_MAX_TIME)
//             {
//                 // Stop.
//                 drive.stop();

//                 // If not to point at the target,
//                 if (!willPointTowardsTarget)
//                 {
//                     // Note pattern completion.
//                     currentState = State.COMPLETE;
//                 }
//                 else // If to point at a target,
//                 {
//                     double angleToTurn = 90.0;

//                     // If we turned clockwise initially,
//                     if (initialTurnAngle > 0)
//                     {
//                         // Turn counterclockwise.
//                         angleToTurn *= -1;
//                     }

//                     drive.AutoTurn(angleToTurn);

//                     currentState = State.TURNING_TOWARDS_TARGET;

//                     // Reset the timestamp.
//                     resetStartTime();
//                 }
//             } // End completion check.
//         }
//         else // Otherwise, we are turning towards the target.
//         {
//             // Check to see if the turn has completed.
//             if (Math.abs(drive.getDegreesToTurn()) < FINAL_ANGLE_TO_TARGET_TOLERANCE || deltaTime > FINAL_TURN_MAX_TIME)
//             {
//                 drive.stop();
                
//                 currentState = State.COMPLETE;
//             }
//         } // End else
//     }

//     // Get whether the pattern has been completed.
//     @Override
//     public boolean getShouldStop()
//     {
//         return currentState == State.COMPLETE;
//     }
//  }

