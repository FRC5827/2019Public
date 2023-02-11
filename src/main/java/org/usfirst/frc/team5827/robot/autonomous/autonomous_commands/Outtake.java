package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

/**
 * A class to handle the hatch manipulation mechanism.
 */

 public class Outtake extends AutonCommand
 {
     public Outtake(double duration)
     {
        super(duration);
     }

     // Start the pattern.
     @Override
     protected void onCommandStart()
     {
        m_robotControl.getHatchManipulator().outtake();
     }
 }