// package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

// /**
//  * An abstract class allowing the management of autonomous commands.
//  * An abstract class was chosen over an interface because not all
//  * commands need commandPeriodic or getShouldStop.
//  * 
//  * Notes:
//  *  - In subclasses:
//  *      - Start: Override onPatternStart().
//  *      - Stop: Override onPatternStop().
//  * 
//  * @author Henry
//  */

// import org.usfirst.frc.team5827.robot.robot_resources.RobotControl;

// public abstract class AutonCommand 
// {
//     protected double m_duration;
//     protected boolean running = false;
//     protected RobotControl m_robotControl = null;

//     // All autonomous commands should have a maximum
//     //run time.
//     public AutonCommand(double duration)
//     {
//         m_duration = duration;
//     }

//     public void setRobotControl(RobotControl robotControl)
//     {
//         m_robotControl = robotControl;
//     }

//     // Note that the command has been started, and run the subclass' 
//     //start code.
//     public final void start()
//     {
//         running = true;

//         // Call the sub-class' implementation (or the default).
//         onCommandStart();
//     }

//     // To be overridden by sub-classes.
//     protected void onCommandStart()
//     {
//         // To be implemented in base classes -- not every command
//         //needs to do something on initialization.
//     }

//     // Whether the command has been started,
//     public boolean getStarted()
//     {
//         return running;
//     }

//     // To be run periodically during the command's execution.
//     public void commandPeriodic() 
//     {
//         // By default, run ArcadeDrive with 0 input.
//         m_robotControl.getDrive().drive(0, 0);
//     }
    
//     // Get whether the command should stop execution.
//     public boolean getShouldStop() 
//     {
//         return false;
//     }

//     // Get the command's duration.
//     public double getDuration()
//     {
//         return m_duration;
//     }

//     // By default, note that the command is not running.
//     //Do not allow sub-classes to override this. It should
//     //be noted that the command is no longer running.
//     public final void stop()
//     {
//         running = false;

//         // Call the sub-class' implementation.
//         onPatternStop();
//     }

//     // Provide an overridable method for stopping the
//     //pattern.
//     protected void onPatternStop()
//     {
//         // By default, stop the drive, if it is accessable.
//         if (m_robotControl != null)
//         {
//             m_robotControl.getDrive().stop();
//         }
//     }
// }
