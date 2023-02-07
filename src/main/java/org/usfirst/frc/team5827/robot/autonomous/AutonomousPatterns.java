/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5827.robot.autonomous;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

import org.usfirst.frc.team5827.robot.autonomous.autonomous_commands.*;

/**
 * 
 * Autonomous Patterns for our Robot
 * @author Matthew
 */

public class AutonomousPatterns 
{
    private Queue<AutonCommand> commands;

    public static enum Pattern { TO_PERPINDICULAR_TARGET, TO_TARGET, TURN_TO_TARGET, GET_HATCH, PLACE_HATCH, TEST_AUTON, LOWER_RIGHT, LOWER_LEFT, CLIMB, EMPTY };

    public Queue<AutonCommand> autonomous(Pattern patternId) {
        commands = new LinkedList<AutonCommand>();

        switch(patternId)
        {
            case TO_PERPINDICULAR_TARGET:
                addCommand(new TurnToTarget(999.9, 2));
                addCommand(new ToPerpendicularTarget(999.9, 18));
                break;
            case TO_TARGET: // TODO Test this.
                // Move to the target's normal
                addCommand(new ToTargetNormal(30.0,
                                    true)); // And turn partially towards the target.

                // Turn towards the target if it is visible.
                addCommand(new TurnToTarget(999.9, 2));
                addCommand(new ToPerpendicularTarget(999.9, 18));
                break;
            case TURN_TO_TARGET: 
                addCommand(new TurnToTarget(999.9, 0.0));
                break;
            case GET_HATCH:
                //addCommand(new Outtake(0.1));
                addCommand(new Extend(0.1));
                addCommand(new TurnToTarget(999.9, 4));
                addCommand(new ToPerpendicularTarget(999.9, 17));
                //addCommand(new DriveDistance(1.5, 2.3));
                //addCommand(new Intake(0.5));
                //addCommand(new Retract(0.1));
                //addCommand(new DriveDistance(2, -3));
                break;
            case PLACE_HATCH:
                //addCommand(new Intake(0.1));
                addCommand(new Extend(0.1));
                addCommand(new TurnToTarget(999.9, 4));
                addCommand(new ToPerpendicularTarget(999.9, 17));
                //addCommand(new DriveDistance(1.5, 2.5));
                //addCommand(new Extend(0.5));
                //addCommand(new Outtake(0.5));
                //addCommand(new Retract(0.1));
                //addCommand(new DriveDistance(2, -3));
                break;
            case LOWER_RIGHT:
                addCommand(new DriveDistance(2, 5));
                addCommand(new Turn(1, -90.0));
                addCommand(new DriveDistance(1, 3));
                addCommand(new Turn(1, 90.0));
                //addCommand(new Retract(0.1));
                addCommand(new TurnToTarget(999.9, 4));
                addCommand(new ToPerpendicularTarget(999.9, 30));
                addCommand(new DriveDistance(1.5, 1.75));
                //addCommand(new Extend(0.5));
                addCommand(new Outtake(0.5));
                //addCommand(new Retract(0.1));
                addCommand(new DriveDistance(2, -3));
                break;
            case LOWER_LEFT:
                addCommand(new DriveDistance(2, 5));
                addCommand(new Turn(1, 90.0));
                addCommand(new DriveDistance(1, 3));
                addCommand(new Turn(1, -90.0));
                addCommand(new TurnToTarget(999.9, 4));
                //addCommand(new Retract(0.1));
                addCommand(new TurnToTarget(999.9, 4));
                addCommand(new ToPerpendicularTarget(999.9, 30));
                addCommand(new DriveDistance(1.5, 1.75));
                //addCommand(new Extend(0.5));
                addCommand(new Outtake(0.5));
                //addCommand(new Retract(0.1));
                addCommand(new DriveDistance(2, -3));
                break;
            case CLIMB:
                addCommand(new Climb());
                break;
            case EMPTY:
                break;
            default:
                // Move to the target's normal
                addCommand(new ToTargetNormal(60.0,
                                    true)); // And turn partially towards the target.
                // Turn towards the target if it is visible.
                addCommand(new TurnToTarget(999.9, 2));
                addCommand(new ToPerpendicularTarget(999.9, 18));
                break;
        }

        // Return the commands list.
        return commands;
    } 

    private void addCommand(AutonCommand cmd)
    {
        commands.add(cmd);
    }
}