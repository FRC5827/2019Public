// package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

// import org.usfirst.frc.team5827.robot.Logging;
// import org.usfirst.frc.team5827.robot.robot_resources.Climber;

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
//  * @author Matthew
//  */

// import org.usfirst.frc.team5827.robot.robot_resources.RobotControl;
// import org.usfirst.frc.team5827.robot.robot_resources.RobotMap;

// import edu.wpi.first.wpilibj.Timer;

// public class Climb extends AutonCommand
// {
//     private int encoderValue;
//     private Climber climber;
//     private Timer timer;
//     private double pauseTime;

//     public Climb()
//     {
//         super(999.0);
//     }

//     //Set up values and timers
//     @Override
//     public void onCommandStart()
//     {
//         climber = m_robotControl.getClimber();
//         encoderValue = climber.getEncoderValue();
//         timer = new Timer();
//         timer.start();
//         m_robotControl.getHatchManipulator().retract();
//         m_robotControl.getShifter().lowGear();
//         m_robotControl.getFlag().lower();
//         Logging.consoleLog("Climb encoderValue: %d", encoderValue);
//         climber.ramp(2);
//         //climber.MotionMagic(RobotMap.Constants.encoderMax);
//     }

//     //TODO:Hold climber up during regular drive like -.05 or -.1
//     //TODO: The final state of the climb 
//     @Override
//     public void commandPeriodic() {
        
//         encoderValue = (climber.getEncoderValue());
//         if(encoderValue < RobotMap.Constants.encoderDiff - 750) {//was 2100
//             climber.climb(0.7);
//         }
//         else if(encoderValue < RobotMap.Constants.encoderDiff){
//             climber.climb(0.3);
//         }
//         else{
//             climber.climb(0.5);
//         }
//         if(timer.get() > 2.0 && encoderValue < RobotMap.Constants.encoderDiff - 140) {//was 2000
//             m_robotControl.getDrive().drive(0.2, 0.0);
//         }
//         if(timer.get() > 1.5) {
//             climber.ramp(0);
//         }
//     }

//     @Override
//     public boolean getShouldStop() {
//         return false;//|| encoderValue > 840;//was 2140
//     }
// }
