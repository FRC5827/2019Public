package org.usfirst.frc.team5827.robot.autonomous.autonomous_commands;

/**
 * An autonomous pattern to allow the robot to turn in autonomous.
 */

 public class Turn extends AutonCommand
 {
     protected double turnDegrees;

     // Use a PID loop to turn to a certain angle.
     public Turn(double duration, double degrees)
     {
        super(duration);

        turnDegrees = degrees;
     }

     // Enable the PID loop and turn.
     @Override
     protected void onCommandStart()
     {
         m_robotControl.getDrive().AutoTurn(turnDegrees);
     }
 }