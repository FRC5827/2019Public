/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5827.robot.robot_resources;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightConnector;
import org.usfirst.frc.team5827.robot.limelight_connector.LimeLightPIDSource;
import org.usfirst.frc.team5827.robot.PIDLoop;
import org.usfirst.frc.team5827.robot.PIDHandler;
import org.usfirst.frc.team5827.robot.RotEncoder;
import org.usfirst.frc.team5827.robot.Logging;
import org.usfirst.frc.team5827.robot.dashboard_manager.*;

/**
 * Drive base for our robot
 * @author Matthew, Pranav
 */
public class Drive {
    //Motors
    public WPI_VictorSPX rightVictor;
    public WPI_TalonSRX rightTalon;
    public WPI_TalonSRX right6;
    public WPI_TalonSRX leftTalon;
    public WPI_TalonSRX left3;
    public WPI_VictorSPX leftVictor;

    //Encoder
    public RotEncoder leftDriveEncoder;
    //Drive base
    public MotorControllerGroup left;
    public MotorControllerGroup right;
    public DifferentialDrive drive;
    //Gyro
    private AHRS gyro;
    //Drive Modes
    public enum DriveMode { CURVATURE_DRIVE, ARCADE_DRIVE };
    private DriveMode driveMode = DriveMode.CURVATURE_DRIVE;
    //PID loop manager.
    private enum PIDLoopNames { LIMELIGHT_TURN, LIMELIGHT_DISTANCE, LIMELIGHT_SKEW, GYRO_TURN, ENCODER_DISTANCE };
    private PIDHandler<PIDLoopNames> pidLoopHandler;
    //Whether drive assist is enabled
    private boolean driveAssist;

    //For Motion Magic
    private int timeout = 0;
    private boolean motionmagic = false;

    //Construct a new Drive base
    public Drive(){
        //initialize motors
        rightTalon = new WPI_TalonSRX(RobotMap.MotorControllerPorts.RightTalon);
        leftTalon = new WPI_TalonSRX(RobotMap.MotorControllerPorts.LeftTalon);
        rightVictor = new WPI_VictorSPX(RobotMap.MotorControllerPorts.RightVictor);
        leftVictor = new WPI_VictorSPX(RobotMap.MotorControllerPorts.LeftVictor);

        //use motors to construct drive base
        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
        left = new MotorControllerGroup(leftTalon);
        right = new MotorControllerGroup(rightTalon);
        drive = new DifferentialDrive(left, right);

        drive.setDeadband(0.0);

        //initialize gyro, encoders, and PID loops
        pidLoopHandler = new PIDHandler<PIDLoopNames>();

        // Gather PID Sources.
        gyro = new AHRS(SPI.Port.kMXP);
        PIDSource limelightAnglePIDSource = new LimeLightPIDSource(LimeLightConnector.AccessibleProperties.X_OFFSET),
                limelightDistancePIDSource = new LimeLightPIDSource(LimeLightConnector.AccessibleProperties.DISTANCE);


        // Constants.
        float angleInputMin = -180.0f,
            angleInputMax = 180.0f,
            angleTolerance = 0.5f;
		
		float distanceInputMin = 0.0f, // Inches.
            distanceInputMax = 360.0f,
            distanceTolerance = 1.0f; // Inches.
            
        double kP = 0.02, kI = 0.00005, kD = 0.05; // The proportional, integral, and differental parts.

        // Enable the drive encoders if available,
        if(RobotMap.EncoderOptions.USE_ENCODERS)
        {
            leftDriveEncoder = new RotEncoder(leftTalon.getSensorCollection());
            pidLoopHandler.addPIDLoop(PIDLoopNames.ENCODER_DISTANCE,
                    new PIDLoop("distance", .000005, 0.0, 0.0, leftDriveEncoder, 1000f));
        }

        // Create the PID loops.
        PIDLoop turnPID = new PIDLoop("gyroturn", 0.01, 0, 0, gyro, -180.0f, 180.0f, 2.0f, true);
        PIDLoop limelightTurn = new PIDLoop("limelightTurn", kP * 1.2, kI * 10, kD, limelightAnglePIDSource, angleInputMin, 
                angleInputMax, angleTolerance, true);
        PIDLoop limelightDistance = new PIDLoop("limelightDistance", kP, 0 /* kI * 2.0 */, 0 /* kD * 2 */, limelightDistancePIDSource, distanceInputMin, 
                distanceInputMax, distanceTolerance, false);

        // Add the PID loops to the PID handler.
        pidLoopHandler.addPIDLoop(PIDLoopNames.GYRO_TURN, turnPID);
        pidLoopHandler.addPIDLoop(PIDLoopNames.LIMELIGHT_TURN, limelightTurn);
        pidLoopHandler.addPIDLoop(PIDLoopNames.LIMELIGHT_DISTANCE, limelightDistance);

        // Enable drive assist.
        //driveAssist = true;

        // Select and enable the gyroscope.
        //pidLoopHandler.selectPIDLoop(PIDLoopNames.GYRO_TURN);
        //pidLoopHandler.setSetpoint(gyro.getYaw());
        //pidLoopHandler.enable();

        //leftTalon.setSensorPhase(true);

        // Dashboard output.
        SmartDashboardOutputter dashboardOutput = new SmartDashboardOutputter();
        SmartDashboardManager.getInstance().addObserver(dashboardOutput);
    }

    public void simpleArcadeDrive(double forwardPower, double turnPower) {
        drive.arcadeDrive(forwardPower, turnPower);
    }

    //Takes input from the joystick and updates values before supplying power to the motors
    public void drive(double forwardPower, double turnPower){
        // Get the PID Loops
        PIDLoop limelightTurn = pidLoopHandler.getPIDLoop(PIDLoopNames.LIMELIGHT_TURN),
                limelightDistance = pidLoopHandler.getPIDLoop(PIDLoopNames.LIMELIGHT_DISTANCE),
                distancePID = pidLoopHandler.getPIDLoop(PIDLoopNames.ENCODER_DISTANCE),
                turnPID = pidLoopHandler.getPIDLoop(PIDLoopNames.GYRO_TURN);
        
        // Encoders are not always available...
        if (RobotMap.EncoderOptions.USE_ENCODERS)
        {
            //updates forward motion value based on PID output. User input overrides PID updates
            if(Math.abs(forwardPower) > 0.01) {
                distancePID.reset();
            }
            else {
                forwardPower += distancePID.getPIDOutput();
            }
        }

        forwardPower += limelightDistance.getPIDOutput();

        turnPower -= limelightTurn.getPIDOutput();

        //updates turn value based on PID output. User input overrides PID updates. Also enables drive assist if no manual turning is detected
        if(Math.abs(turnPower) > 0.01) {
            driveAssist = false;
            turnPID.reset();
        }
        else {
            if(!driveAssist) {
                //driveAssist = true;
                //turnPID.reset();
                //turnPID.setSetpoint(gyro.getYaw());
                //turnPID.enable();
            }
            turnPower += turnPID.getPIDOutput();
        }

        // Determine whether to turn in place.
        boolean toTurnInPlace = false;
        if (Math.abs(forwardPower) < 0.04 && limelightTurn.getPIDOutput() == 0.0 && turnPID.getPIDOutput() == 0.0)
        {
            toTurnInPlace = true;
            double floorRotate = 0.28f;
            double rangeRotate = 1 - floorRotate;
            if (Math.abs(turnPower) > 0.02)
            {
                turnPower = (turnPower > 0) ? turnPower * rangeRotate + floorRotate
                        : turnPower * rangeRotate - floorRotate;
            }
        }

        //drives using updated values
        if (driveMode == DriveMode.CURVATURE_DRIVE)
        {
            drive.curvatureDrive(forwardPower, -turnPower, toTurnInPlace);
        }
        else
        {
            // This segement taken from the 2018 code.
            //Remove it if not wanted.

            // Adjust final values to match power requirements of drive system
            // We want 0..1 inputs to map to 0..1 output power but friction means
            // that we need to provide a floor amount of current just to move.
            double floorSpeed = 0.3f;
            double rangeSpeed = 1 - floorSpeed;
            double floorRotate = 0.28f;
            double rangeRotate = 1 - floorRotate;
            double adjustedSpeed = forwardPower;
            double adjustedRotation = turnPower * 0.8;
            if (Math.abs(forwardPower) > 0.05)
            {
                adjustedSpeed = (forwardPower > 0) ? forwardPower * rangeSpeed + floorSpeed
                        : forwardPower * rangeSpeed - floorSpeed;
            }
            
            if (Math.abs(turnPower) > 0.05)
            {
                adjustedRotation = (turnPower > 0) ? turnPower * rangeRotate + floorRotate
                        : turnPower * rangeRotate - floorRotate;
            }
            // Drive using the adjusted speed and rotation.
            drive.arcadeDrive(adjustedSpeed, -adjustedRotation);
        }
    }

    //Enables PID loop to automatically turn a certain angle
    public void AutoTurn(double degrees){
        // Select the turning loop.
        pidLoopHandler.selectPIDLoop(PIDLoopNames.GYRO_TURN);

        //get our target orientation
        double targetOrientation = gyro.getYaw() + degrees;
        //adjust orientation so it is within our range of -180 to 180
        if(targetOrientation > 180) {
            targetOrientation -= 360;
        }
        if(targetOrientation < -180) {
            targetOrientation += 360;
        }
        //set our setpoint and enable PID loop
        pidLoopHandler.setSetpoint(targetOrientation);
        pidLoopHandler.enable();

        driveAssist = true; // The gyroscope is being used for navigation.
    }
    
    //Enables PID loop to automatically move a certain distance
    public void distanceDrive(double feet)
    {
        //convert value from feet into encoder ticks
        double ticks = -feetToTicks(feet) * 7.5;

        if (!RobotMap.EncoderOptions.USE_ENCODERS)
        {
            Logging.consoleLog("Warning: Distance Drive attempted with no enabled encoders!");

            return;
        }

        // Deselect any previously selected PID loops.
        pidLoopHandler.deselectPIDLoop();

        // Note that multiple loops will now be selected.
        pidLoopHandler.notePotentialForMultiplePIDLoopsSelected();

        //reset encoder value to 0
        leftDriveEncoder.reset();

        // Get the PID loops.
        PIDLoop distancePID = pidLoopHandler.getPIDLoop(PIDLoopNames.ENCODER_DISTANCE),
                anglePID = pidLoopHandler.getPIDLoop(PIDLoopNames.GYRO_TURN);

        //reset PID loops
        distancePID.reset();
        anglePID.reset();

        //set our setpoint and enable PID loops
        distancePID.setSetpoint(ticks);
        anglePID.setSetpoint(gyro.getYaw());

        distancePID.enable();
        anglePID.enable();

        //ramp up motors to prevent jerking
        leftTalon.configOpenloopRamp(0.25);
        rightTalon.configOpenloopRamp(0.25);
    }


    //Vision methods
    /**
	 * Start drive using the limelight camera.
	 */
	public void pointAtTargetDrive()
	{
        pidLoopHandler.selectPIDLoop(PIDLoopNames.LIMELIGHT_TURN);
        pidLoopHandler.setSetpoint(0.0);
        pidLoopHandler.enable();
	}

	/**
	 * Drive to a specific distance from the target.
	 * @param distanceToAchieveFeet The distance from the target to achieve, in feet. This is converted to inches.
	 */
	public void distanceToTargetDrive(double distanceToAchieveFeet)
	{
		// Convert units of arguments,
		double distanceToAchieveInches = distanceToAchieveFeet * 12.0; // 12 inches / foot
        
        // Deselect any previously selected PID loops.
        pidLoopHandler.deselectPIDLoop();

        // Note that multiple loops will now be selected.
        pidLoopHandler.notePotentialForMultiplePIDLoopsSelected();

        // Get the PID Loops
        PIDLoop limelightTurn = pidLoopHandler.getPIDLoop(PIDLoopNames.LIMELIGHT_TURN),
                limelightDistance = pidLoopHandler.getPIDLoop(PIDLoopNames.LIMELIGHT_DISTANCE);

		// Set desired outputs.
		limelightTurn.setSetpoint(0.0);
		limelightDistance.setSetpoint(distanceToAchieveInches);

		// Enable these.
		limelightTurn.enable();
        limelightDistance.enable();

        //ramp up motors to prevent jerking
        leftTalon.configOpenloopRamp(0.25);
        rightTalon.configOpenloopRamp(0.25);
    }
    
    // Stop driving.
    public void stop()
    {
        // Deselect any active PID loops.
        pidLoopHandler.runCommandForAll(PIDHandler.Command.RESET);
        pidLoopHandler.deselectPIDLoop();

        driveAssist = false;
    }

    //For testing individual motors
    public void setLeft3() {
        left3.set(ControlMode.PercentOutput, 0.5);
    }
    public void setLeft4() {
        stopMotors();
        leftTalon.set(ControlMode.PercentOutput, 0.5);
    }
    public void setRight5() {
        stopMotors();
        rightVictor.set(ControlMode.PercentOutput, 0.5);
    }
    public void setLeft2() {
        stopMotors();
        leftVictor.set(ControlMode.PercentOutput, 0.5);
    }
    public void setRight6() {
        right6.set(ControlMode.PercentOutput, 0.5);
    }
    public void setRight1() {
        stopMotors();
        rightTalon.set(ControlMode.PercentOutput, 0.5);
    }
    public void stopMotors() {
        rightTalon.set(ControlMode.PercentOutput, 0);
        rightVictor.set(ControlMode.PercentOutput, 0);
        leftTalon.set(ControlMode.PercentOutput, 0);
        leftVictor.set(ControlMode.PercentOutput, 0);
    }

    // Get the remaining angle
    //between the target orientation
    //and the current.
    public double getDegreesToTurn()
    {
        PIDLoop gyroLoop = pidLoopHandler.getPIDLoop(PIDLoopNames.GYRO_TURN);
        double toTurn = gyro.getAngle() - gyroLoop.getSetpoint();

        return toTurn;
    }

    // Get the remaining distance to travel.
    //This uses the encoders and the distance PID loop
    //to determine the distance in feet to travel a set
    //destination.
    public double getDistanceToTravel()
    {
        if(!RobotMap.EncoderOptions.USE_ENCODERS)
        {
            Logging.consoleLog("Use of this method requires encoders.");

            // TODO Throw an exception here?

            return 0.0;
        }

        PIDLoop distancePID = pidLoopHandler.getPIDLoop(PIDLoopNames.ENCODER_DISTANCE);

        double ticks = leftDriveEncoder.pidGet(); // What the encoder would give the PID loop.
        double goalTicks = distancePID.getSetpoint();

        double remainingTicks = goalTicks - ticks;
        double remainingFeet = ticksToFeet(remainingTicks);

        return remainingFeet;
    }

    // Convert a measurment in feet into
    //ticks (for use with encoders).
    private double feetToTicks(double feet)
    {
        return (((feet*12)/RobotMap.Constants.wheelCircumference)*4096.0);
    }

    // Convert a measurement in ticks into
    //feet.
    private double ticksToFeet(double ticks)
    {
        return ticks / 4096.0 * RobotMap.Constants.wheelCircumference / 12.0;
    }

    // Set the drive mode.
    public void setDriveMode(DriveMode mode)
    {
        driveMode = mode;
    }

    private class SmartDashboardOutputter implements DashboardOutputter
    {
        public void initializeDashboard()
        {
            SmartDashboard.setDefaultNumber("Gyro Yaw", 999.0);
        }

        public void updateDashboard()
        {
            SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        }
    }

    public void setBrakeMode(boolean toBrake) {
        if(toBrake) {
            leftTalon.setNeutralMode(NeutralMode.Brake);
            rightTalon.setNeutralMode(NeutralMode.Brake);
            leftVictor.setNeutralMode(NeutralMode.Brake);
            rightVictor.setNeutralMode(NeutralMode.Brake);
        }
        else {
            leftTalon.setNeutralMode(NeutralMode.Coast);
            rightTalon.setNeutralMode(NeutralMode.Coast);
            leftVictor.setNeutralMode(NeutralMode.Coast);
            rightVictor.setNeutralMode(NeutralMode.Coast);
        }
    }
}