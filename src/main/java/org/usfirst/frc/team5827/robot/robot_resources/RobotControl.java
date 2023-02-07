package org.usfirst.frc.team5827.robot.robot_resources;

/**
 * A class to bundle objects used to control the robot.
 * 
 * @author Henry
 */

 public class RobotControl
 {
    // Resurce member variables.
    private Drive drive;
    private OperatorControl operator;
    private HatchPanelManipulator hatch;
    private Shifter shifter;
    private Climber climb;
    private Flag flag;

    // Initialize resources.
    public RobotControl()
    {
        drive = new Drive();
        operator = new OperatorControl();

        // See RobotMap.java for solenoid identifiers.
        hatch = new HatchPanelManipulator();
        shifter = new Shifter();
        climb = new Climber();
        flag = new Flag();
     }

    // Accessors:

    public Drive getDrive()
    {
        return drive;
    }

    public OperatorControl getOperatorControl()
    {
        return operator;
    }

    public HatchPanelManipulator getHatchManipulator()
    {
        return hatch;
    }

    public Shifter getShifter() {
        return shifter;
    }
     
    public Climber getClimber() {
        return climb;
    }

    public Flag getFlag() {
        return flag;
    }
 }