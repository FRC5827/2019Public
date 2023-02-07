package org.usfirst.frc.team5827.robot.game_segment_manager;

/**
 * A superclass to allow direct handling of AutonomousPatterns
 * and teleop data to be done outside Robot.java.
 * 
 * Goal:
 *  - Classes can extend GameSegmentManager and be used by
 *    Robot.java during different parts of the game.
 *  - To simply Robot.java by moving lower-level code elsewhere.
 * 
 * Notes:
 *  - Like SmartDashboardManager.java, this class might just
 *    add unnecessary complexity. Be careful of this. If it
 *    complicates the code significantly, replace it.
 * 
 * @author Henry
 */

 import org.usfirst.frc.team5827.robot.robot_resources.RobotControl;

 public abstract class GameSegmentManager
 {
     // Protected to allow acces by base classes.
     protected RobotControl robotResourceManager;

     // Initialize protected variables.
     public GameSegmentManager(RobotControl robotResources)
     {
        robotResourceManager = robotResources;
     }

     public abstract void onModeInit();
     public abstract void periodic(); // To be called periodically when enabled.
     public abstract void onModeDisable(); // To be called in disabledInit,
                                  // if this manager was enabled.
 }