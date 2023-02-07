/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5827.robot.robot_resources;

/**
 * Add your docs here.
 */
public class RobotMap {
    public static class MotorControllerPorts {
        public static int RightTalon = 1;
        public static int LeftTalon = 2;
        public static int RightVictor = 3;
        public static int LeftVictor = 4;
        public static int RightClimb = 5;
        public static int LeftClimb = 6;
    }

    //encoder min: -171
    //encoder tip: 1540
    //encoder max: 2140

    // Options for the solenoids, like the shifter.
    //Ordering:
    //The hatch panel manipulator switch is on top,
    //below that is the shifter, and below that is the extender.
    public static class SolenoidOptions
    {
        public static final int HATCH_ID = 8,
                                 HATCH_FORWARD_CHANNEL = 4,
                                 HATCH_BACKWARD_CHANNEL = 5,

                                 SHIFTER_ID = 8,
                                 SHIFTER_FORWARD_CHANNEL = 2,
                                 SHIFTER_BACKWARD_CHANNEL = 3,
                                 
                                 EXTENDER_ID = 8,
                                 EXTENDER_FORWARD_CHANNEL = 6,
                                 EXTENDER_BACKWARD_CHANNEL = 7;

        public static final boolean USE_SOLENOIDS = true;
    }

    public static class EncoderOptions
    {
        public static final boolean USE_ENCODERS = true;
    }

    public static class Joysticks {
        public static int Joystick = 0;
    }
    public static class Constants {
        public static double wheelCircumference = 6 * Math.PI,
                limelightDistanceTolerance = 2.5;

        public static int encoderMin = 2655;
        public static int encoderMax = 4940;
        public static int encoderDiff = encoderMax - encoderMin;
    }
}
