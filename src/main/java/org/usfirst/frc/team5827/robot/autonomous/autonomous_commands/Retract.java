package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

/**
 * A class to handle the hatch intake mechanism.
 */

 public class Retract extends AutonCommand
 {
     public Retract(double duration)
     {
        super(duration);
     }

     // Start the pattern.
     @Override
     protected void onCommandStart()
     {
        m_robotControl.getHatchManipulator().retract();
     }
 }