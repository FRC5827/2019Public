// package org.usfirst.frc.team5827.robot.game_segment_manager;

// import java.util.Queue;

// /**
//  * A class to manage the autonomous period of the game.
//  * 
//  * Goal:
//  *  - To simplify the long autonomousPeriodic method of Robot.java.
//  * 
//  * Notes:
//  *  - This class is a GameSegmentManager.
//  *  - It should be used by Robot.java.
//  *  - The addition of this class might make the code base more,
//  *    not less, complicated. If so, please remove this.
//  * 
//  * @author Henry
//  */

// import edu.wpi.first.wpilibj.Timer;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// import org.usfirst.frc.team5827.robot.autonomous.AutonomousPatterns;
// import org.usfirst.frc.team5827.robot.autonomous.AutonomousPatterns.Pattern;
// import org.usfirst.frc.team5827.robot.autonomous.autonomous_commands.AutonCommand;

// import org.usfirst.frc.team5827.robot.dashboard_manager.*;

// import org.usfirst.frc.team5827.robot.robot_resources.RobotControl;
// import org.usfirst.frc.team5827.robot.robot_resources.Shifter;
// import org.usfirst.frc.team5827.robot.robot_resources.Drive;
// import org.usfirst.frc.team5827.robot.robot_resources.HatchPanelManipulator;

// /*
//  Public Methods:
//     public AutonomousManager();
//     public void periodic();
//     public void onModeInit();
//     public void onModeDisable();
// */

// public class AutonomousManager extends GameSegmentManager
// {
//     private Queue<AutonCommand> autonomousCommands;

//     private Timer autonTimer;

//     private AutonomousPatterns patterns;
//     private AutonomousPatterns.Pattern currentPattern;

//     private AutonomousManagerDashboardOutputter userOutput;

//     private static AutonomousPatterns.Pattern defaultPattern = AutonomousPatterns.Pattern.TEST_AUTON;

//     // Create a new AutonomousManager,
//     //initialize timers and patterns.
//     public AutonomousManager(RobotControl robotResources)
//     {
//         super(robotResources);

//         patterns = new AutonomousPatterns();
//         autonTimer = new Timer();
//         userOutput = new AutonomousManagerDashboardOutputter();

//         // Allow the user output class' updateDashboard to
//         //be called periodically.
//         SmartDashboardManager.getInstance().addObserver(userOutput);
//     }

//     public void periodic()
//     {
//         stopAutonCommandIfNecessary();

//         // Only consider new commands if there
//         //are commands left to run.
//         if (autonomousCommands != null && autonomousCommands.size() > 0)
//         {
//             startAutonCommandIfNecessary();

//             // Run the current command's periodic segement.
//             AutonCommand currentCommand = getAutonCommand();
//             currentCommand.commandPeriodic();
//         }
//     }
//     // Initialize drive settings.
//     public void configureDrive()
//     {
//         Drive drive = robotResourceManager.getDrive();
//         drive.setDriveMode(Drive.DriveMode.ARCADE_DRIVE);
//         Shifter shift = robotResourceManager.getShifter();
//         shift.lowGear();
//     }

//     // Start the autonomous timer and select a pattern.
//     public void onModeInit()
//     {
//         //Logging.consoleLog("auton mode init start");

//         //Extend Hatch Manipulator
//         HatchPanelManipulator hatch = robotResourceManager.getHatchManipulator();
//         hatch.extend();
//         hatch.intake();

//         //
//         //Logging.consoleLog("auton mode init end");
//     }
//     // Used by TeleopManager
//     public void onModeDisable()
//     {
//         stopAutonCommand();

//         // Stop the drive.
//         robotResourceManager.getDrive().stop();
//     }

//     // A class to output periodic data to the smart dashboard regarding autonomous.
//     //This includes handling choosers, etc.
//     protected class AutonomousManagerDashboardOutputter implements DashboardOutputter
//     {
//         private SendableChooser<AutonomousPatterns.Pattern> autoOption;
//         private AutonomousPatterns.Pattern lastSelectedPattern;

//         public void initializeDashboard()
//         {
//             autoOption = new SendableChooser<AutonomousPatterns.Pattern>();

//             // Add every pattern as an autonomous option.
//             for(AutonomousPatterns.Pattern pattern : AutonomousPatterns.Pattern.values())
//             {
//                 autoOption.addOption(pattern.toString(), pattern);
//             }

//             // Set the default.
//             autoOption.setDefaultOption(defaultPattern.toString(), defaultPattern);

//             // Add the chooser to the dashboard.
//             SmartDashboard.putData("Autonomous Pattern", autoOption);
//             SmartDashboard.putString("Current Auto Pattern", defaultPattern.toString());

//             // Note the last selected pattern.
//             lastSelectedPattern = defaultPattern; // TODO Check that this is true.
//         }

//         public void updateDashboard()
//         {
//             if (currentPattern != null)
//             {
//                 SmartDashboard.putString("Current Auto Pattern", currentPattern.toString());
//             }
//             else
//             {
//                 SmartDashboard.putString("Current Auto Pattern", "null");
//             }

//             AutonomousPatterns.Pattern wantedPattern = autoOption.getSelected();

//             // If the wanted pattern has changed AND it isn't the current pattern,
//             if (wantedPattern != lastSelectedPattern && wantedPattern != currentPattern)
//             {
//                 // Select that pattern.
//                 selectPattern(wantedPattern);

//                 lastSelectedPattern = wantedPattern;
//             }
//         }

//         public void onPatternChange()
//         {
//             // TODO Investigate whether this changing of the default pattern actually does anything.
//             autoOption.setDefaultOption(currentPattern.toString(), currentPattern);

//             // Update the new selected pattern.
//             lastSelectedPattern = autoOption.getSelected();
//         }
//     }

//     // Select an autonomous pattern to be used.
//     public void selectPattern(AutonomousPatterns.Pattern patternToUse)
//     {
//         // Stop any previously running commands.
//         stopAutonCommand();

//         autonomousCommands
//             = patterns.autonomous(patternToUse);
        
//         currentPattern = patternToUse;

//         // Notify the output of a potential pattern change.
//         userOutput.onPatternChange();
//     }

//     // Start a given command and the timer.
//     protected void startAutonCommandIfNecessary(AutonCommand operation)
//     {
//         // Stop if the operation was already started.
//         if (operation.getStarted())
//         {
//             return;
//         }

//         // Ensure the operation has access to resources.
//         operation.setRobotControl(robotResourceManager);

//         // Start the operation.
//         operation.start();

//         // Reset the timer.
//         autonTimer.reset();
//         autonTimer.start();
//     }

//     // Start the current command.
//     protected void startAutonCommandIfNecessary()
//     {
//         AutonCommand nextCommand = getAutonCommand();

//         // Stop if there are no more commands.
//         if (nextCommand == null)
//         {
//             return;
//         }

//         startAutonCommandIfNecessary(nextCommand);
//     }

//     // Stop the last command, regardless of whether it was running.
//     protected void stopAutonCommand()
//     {
//         // If a pattern is already running,
//         if (autonomousCommands != null)
//         {
//             AutonCommand lastCommand = getAutonCommand(); // Get the top element.
            
//             // Stop if there are no more commands.
//             if (lastCommand == null)
//             {
//                 return;
//             }

//             // Tell the command to stop.
//             lastCommand.stop();

//             // Remove the command from the list of commands to handle.
//             autonomousCommands.remove();
//         }
//     }

//     protected void stopAutonCommandIfNecessary()
//     {
//         // If a pattern is already running,
//         if (autonomousCommands != null)
//         {
//             AutonCommand lastCommand = getAutonCommand(); 

//             // Stop if there are no more commands.
//             if (lastCommand == null)
//             {
//                 return;
//             }
            
            
//             if (lastCommand.getStarted() // So long as the command has been started,
//                     && (lastCommand.getDuration() <= autonTimer.get() // If the duration has been exceeded (or met),
//                     || lastCommand.getShouldStop())) // Or the command has finished.
//             {
//                 stopAutonCommand();
//             }
//         }
//     }

//     // Get the top element -- the current command.
//     protected AutonCommand getAutonCommand()
//     {
//         // So long as there are commands,
//         if (autonomousCommands.size() > 0)
//         {
//             AutonCommand currentCommand = autonomousCommands.peek();

//             return currentCommand;
//         }

//         return null;
//     }
// }