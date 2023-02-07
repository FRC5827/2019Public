package org.usfirst.frc.team5827.robot.limelight_connector;

import java.util.Arrays;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.Comparator;
import java.util.Date;

import org.usfirst.frc.team5827.robot.dashboard_manager.DashboardOutputter;
import org.usfirst.frc.team5827.robot.dashboard_manager.SmartDashboardManager;

import org.usfirst.frc.team5827.robot.MathHelper;
import org.usfirst.frc.team5827.robot.Logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** 
 * A wrapper and test class for the limelight.
 * LimeLightConnector is static and will auto-initialize
 * when methods are called.
 * 
 * The limelight has a 59.6 degree horizontal field of view,
 * and a 320 px by 240 px screen size.
 * 
 * See https://media.readthedocs.org/pdf/limelight/latest/limelight.pdf
 * for limelight docs.
*/


public class LimeLightConnector
{
    private static NetworkTable networkConnection; // The table this class wraps.
    private static NetworkTableEntry tx, ty, ta, tv, ts, tcornX, tcornY, thor, // thor is the horizontal width of the rough bounding box.
            snapshot, ledMode, camMode, latency,
            setPipelineNetworkEntry, getPipelineNetworkEntry; // These will be instantiated. Data can be gotten by calling <variable name>.get<Type>(<default value>);
    private static boolean initialized = false; // Whether the initializer has been called. Allows this class to be static.
    public static double CAMERA_HEIGHT = 45.75,//15.5;
                        TARGET_HEIGHT = 31.5, // The height of the target from the ground.
                        TARGET_WIDTH = 15.0, 
                        TARGET_TOP_WIDTH = 11.8725,
                        CAMERA_ANGLE = -22.31459, //-25.00697820, //-9.39042644, // In degrees.
                        FOCAL_LENGTH = 1.0,
                        SCREEN_WIDTH = 320.0,
                        SCREEN_HEIGHT = 240.0,
                        ERROR_ANGLE = 999.0,
                        FOV_X = 59.6,
                        FOV_Y = 49.7,
                        VIEW_PLANE_WIDTH = 2.0 * Math.tan(FOV_X / 2.0 * Math.PI / 180.0),
                        VIEW_PLANE_HEIGHT = 2.0 * Math.tan(FOV_Y / 2.0 * Math.PI / 180.0);

    public static final LimeLightPipeline DEFAULT_PIPELINE = LimeLightPipeline.CLOSEST;

    //x,y,z of object. 0 point (origin) is center of top points of angled reflective tape
    private static double[] m_LbotLeft = {-7.31338530, -5.32481202, 0.0};
    private static double[] m_LtopLeft = {-5.93629528, 0.0, 0.0};
    private static double[] m_LbotRight = {-5.37709002, -5.81335205, 0.0};
    private static double[] m_LtopRight = {-4, -0.50076008, 0.0};
    private static double[] m_RtopLeft = {4, -0.50076008, 0.0};
    private static double[] m_RbotLeft = {5.37709002, -5.81335205, 0.0};
    private static double[] m_RtopRight = {5.93629528, 0.0, 0.0};
    private static double[] m_RbotRight = {7.31338530, -5.32481202, 0.0};
    
    private static MatOfPoint3f m_objectPoints;

    // Limelight v2 camera matrix is [ 2.5751292067328632e+02, 0., 1.5971077914723165e+02, 0.,2.5635071715912881e+02, 1.1971433393615548e+02, 0., 0., 1. ]
    // Limelight v2 distortion coefficients are [ 2.9684613693070039e-01, -1.4380252254747885e+00, -2.2098421479494509e-03, -3.3894563533907176e-03, 2.5344430354806740e+00 ]
    private static Mat m_cameraMatrix;          // camera matrix input into solvePnP using values for Limelight v2
    private static MatOfDouble m_distCoeffs;    // distortion coefficients of Limelight v2
    private static double m_k1 =  2.9684613693070039e-01,
                          m_k2 = -1.4380252254747885e+00,
                          m_p1 = -2.2098421479494509e-03,
                          m_p2 = -3.3894563533907176e-03,
                          m_k3 =  2.5344430354806740e+00;

    public enum AccessibleProperties
    {
        HAS_TARGET, X_OFFSET, Y_OFFSET,
        AREA, DISTANCE, SKEW
    }

    public enum CornerTypes
    {
        TOP_LEFT, BOTTOM_LEFT, TOP_RIGHT, BOTTOM_RIGHT
    }

    /** 
     * This method connects to the lime light's network table
     * and initializes network table entries. It is called automatically
     * by all accessors and modifiers.
    */
    public static void initializeIfNeeded()
    {
        if (!initialized) // If not already initialized,
        {
            networkConnection = NetworkTableInstance.getDefault().getTable("limelight");

            tx = networkConnection.getEntry("tx");
            ty = networkConnection.getEntry("ty");
            ta = networkConnection.getEntry("ta");
            tv = networkConnection.getEntry("tv");
            ts = networkConnection.getEntry("ts"); // Skew
            thor = networkConnection.getEntry("thor");
            tcornX = networkConnection.getEntry("tcornx");
            tcornY = networkConnection.getEntry("tcorny");
            latency = networkConnection.getEntry("tl");
            camMode = networkConnection.getEntry("camMode");
            ledMode = networkConnection.getEntry("ledMode");
            snapshot = networkConnection.getEntry("snapshot");
            setPipelineNetworkEntry = networkConnection.getEntry("pipeline");
            getPipelineNetworkEntry = networkConnection.getEntry("getpipe");

            // set object in world coordinates for use by solvePnP
            m_objectPoints = new MatOfPoint3f(
                new Point3(m_LbotLeft),
                new Point3(m_LtopLeft),
                //new Point3(m_LbotRight),
                new Point3(m_LtopRight),
                new Point3(m_RtopLeft),
                //new Point3(m_RbotLeft),
                new Point3(m_RtopRight),
                new Point3(m_RbotRight));
            
            // set camera inputs for solvePnP (focal lengths in pixel units and image center)
            m_cameraMatrix = new Mat(3, 3, CvType.CV_64F, Scalar.all(0.0));
            m_cameraMatrix.put(0, 0, 2.5751292067328632e+02);
            m_cameraMatrix.put(0, 2, 1.5971077914723165e+02);
            m_cameraMatrix.put(1, 1, 2.5635071715912881e+02);
            m_cameraMatrix.put(1, 2, 1.1971433393615548e+02);
            m_cameraMatrix.put(2, 2, 1.0);

            m_distCoeffs = new MatOfDouble(m_k1, m_k2, m_p1, m_p2, m_k3);

            // Enable limelight output to the smart dashboard.

            LimeLightSmartDashboardOutputter dashboardOutput = new LimeLightSmartDashboardOutputter();
            SmartDashboardManager.getInstance().addObserver(dashboardOutput); // Include the output object as one to display dashboard information.

            initialized = true;
        }
    }

    /** 
     * Get tx from the network table.
     * This is the horizontal offset to the target.
     * It is measured in degrees.
    */
    public static double getXOffset()
    {
        initializeIfNeeded();

        return tx.getDouble(0.0);
    }

    /** 
     * Get the vertical offset in degrees from the
     * limelight's calibrated zero point (with crosshairs)
     * to the target.
     */
    public static double getYOffset()
    {
        initializeIfNeeded();

        return ty.getDouble(0.0);
    }

    /**
     * Get the skew of the target: The screen rotation (z rotation) of the
     * projection of the target onto the screen.
     * @return Zero or the skew of the target. Zero is returned if there is no target or
     * the limelight was not connected to.
     */
    public static double getSkew()
    {
        initializeIfNeeded();

        return ts.getDouble(0.0);
    }

    /** 
     * Auto-initializes and returns the area
     * of the detected target, from 0% of the
     * image to 100% of the image.
    */
    public static double getTargetArea()
    {
        initializeIfNeeded();

        return ta.getDouble(0.0);
    }

    /**
     * Get the coordinates of a specified corner.
     * @param cornerName The name of the corner to find.
     * @return The specified corner, as a Point2D.
     */
    public static boolean getCorner(CornerTypes cornerName, MathHelper.Point2D resultPoint)
    {
        initializeIfNeeded();

        // Get the raw corner x and y values from the Limelight.
        double[] xLocations = tcornX.getDoubleArray(new double[0]);
        double[] yLocations = tcornY.getDoubleArray(new double[0]);

        if (xLocations.length < 4 || xLocations.length != yLocations.length
             || xLocations.length % 2 != 0)
        {
            return false;
        }

        // Whether the corner to be considered is better than whichever was previously selected.
        boolean betterThanPreviouslySelected = false;

        double x, y, selectedX, selectedY;

        selectedX = xLocations[0];
        selectedY = yLocations[0];

        // Set an initial result.
        MathHelper.Point2D result = new MathHelper.Point2D(selectedX, selectedY);

        // First divide into upper corners and lower corners.
        MathHelper.Point2D[] leftCorners = new MathHelper.Point2D[xLocations.length / 2];
        MathHelper.Point2D[] rightCorners = new MathHelper.Point2D[yLocations.length / 2];

        MathHelper.Point2D[] cornersArray = new MathHelper.Point2D[xLocations.length];



        // For every point,
        for (int i = 0; i < xLocations.length; i++)
        {
            x = xLocations[i];
            y = yLocations[i];

            cornersArray[i] = new MathHelper.Point2D(x, y);
        }

        // Arrays.sort sorts in ascending order.
        Arrays.sort(cornersArray, new Comparator<MathHelper.Point2D>()
        {
            public int compare(MathHelper.Point2D objectA, MathHelper.Point2D objectB)
            {
                double deltaX = objectA.getX() - objectB.getX();

                int comparison = 0;

                if(deltaX > 0.0)
                {
                    comparison = 1;
                }
                else if(deltaX < 0.0)
                {
                    comparison = -1;
                }

                return comparison;
            }
        });

        // Put the first two into the upperCorners array, and the last two into the lowerCorners array.
        // Upper corners.
        for(int i = 0; i < cornersArray.length / 2; i++)
        {
            leftCorners[i] = cornersArray[i];
            rightCorners[i] = cornersArray[cornersArray.length - 1 - i];
        }

        // Select one of the two corners arrays.
        boolean wantsLeftCorner = (cornerName == CornerTypes.TOP_LEFT || cornerName == CornerTypes.BOTTOM_LEFT);
        MathHelper.Point2D[] arrayToFilter = wantsLeftCorner && leftCorners.length > 0 ? leftCorners : rightCorners;

        // By default, select the 1st corner.
        result = arrayToFilter[0];
        selectedX = result.getX();
        selectedY = result.getY();

        // Select the corner.
        for(int i = 1; i < arrayToFilter.length; i++)
        {
            x = arrayToFilter[i].getX();
            y = arrayToFilter[i].getY();

            switch(cornerName)
            {
                case BOTTOM_LEFT:
                case BOTTOM_RIGHT:
                    betterThanPreviouslySelected = y < selectedY;
                    break;
                
                case TOP_LEFT:
                case TOP_RIGHT:
                    betterThanPreviouslySelected = y > selectedY;
                    break;
            }

            // If better than the previous,
            if(betterThanPreviouslySelected)
            {
                selectedX = x;
                selectedY = y;

                result = arrayToFilter[i];
            }
        }

        // Set output components
        resultPoint.setX(result.getX());
        resultPoint.setY(result.getY());

        // Note success.
        return true;
    }

    /**
     * Get the horizontal length of the rough bounding box of the
     * target.
     * @return The width.
     */
    public static double getTargetScreenWidth()
    {
        initializeIfNeeded();

        return thor.getDouble(0.0);
    }

    /** 
     * Return whether the limelight has a target, 
     * auto-initializes.
    */
    public static boolean hasTarget()
    {
        initializeIfNeeded();

        return (int)(tv.getDouble(0.0) + 0.5) == 1;
    }

    /**
     * Approximate the distance to the target.
     * See Limelight docs, section 7.1.
     * @param targetHeight The known height of the target, in the same units of the cameraHeight and distance
     * @param cameraHeight The known height of the camera.
     * @param cameraAngle The known angle of the camera, with zero degrees being parallel to the ground.
     * @param targetYAngle The y-rotation of the target.
     * @return The distance in the same units of targetHeight and cameraHeight.
     */
    public static double getDistance(double targetHeight, double cameraHeight, double cameraAngle, double targetYAngle)
    {
        initializeIfNeeded();

        double angle = targetYAngle + cameraAngle;
        double deltaH = targetHeight - cameraHeight;

        angle *=  Math.PI / 180.0; // Convert the angle into radians.

        double cotAngle;

        double tanAngle = Math.tan(angle);

        // Prevent division by zero.
        if(tanAngle != 0.0)
        {
            cotAngle = 1 / tanAngle;
        }
        else
        {
            cotAngle = 0;
        }

        // a / o * o gives a. a/o is cot(angle).
        double distance = deltaH * cotAngle;

        return distance;
    }

    /**
     * Calculate the camera's angle given the distance to the target.
     * Ensure TARGET_HEIGHT and CAMERA_HEIGHT are properly set.
     * @param targetDistance The known distance to the target.
     * @return Returns a camera's angle in degrees.
     */
    public static double calcCameraAngle(double targetDistance)
    {
        initializeIfNeeded();

        double deltaH = TARGET_HEIGHT - CAMERA_HEIGHT;

        // Calculate the inverse camera angle.
        double cameraAngle = Math.atan(deltaH / targetDistance);

        // Convert this to degrees.
        cameraAngle *= 180.0 / Math.PI;

        // Now that the camera angle is in degrees, 
        //subtract the y offset.
        cameraAngle -= getYOffset();

        // Return this angle.
        return cameraAngle;
    }

    public static double[] getXcorn()
    {
        initializeIfNeeded();

        return tcornX.getDoubleArray(new double[0]);
    }

    public static double[] getYcorn()
    {
        initializeIfNeeded();

        return tcornY.getDoubleArray(new double[0]);
    }

    public static double angleOfTarget()
    {
        // resources:
        // https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#solvepnp
        // https://docs.opencv.org/2.4/modules/gpu/doc/operations_on_matrices.html
        // https://docs.opencv.org/java/2.4.2/org/opencv/core/Core.html
        // https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp
        // https://www.chiefdelphi.com/t/limelight-real-world-camera-positioning/343941
        // https://www.chiefdelphi.com/t/finding-camera-location-with-solvepnp/159685
        // https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf
        // https://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
        // https://stackoverflow.com/questions/13823296/converting-opencv-rotation-and-translation-vectors-to-xyz-rotation-and-xyz-posit

        initializeIfNeeded();

        double angleToTarget = 0, angleOfTarget = 0, distance = 0;
        Mat rotationMatrix = new Mat();
        Mat pzero_world = new Mat();
        double[] xcorners, ycorners;
        Point[] pointArray = new Point[8];

        xcorners = getXcorn();
        ycorners = getYcorn();

        if (xcorners.length != 8 || ycorners.length != 8) {
            //Logging.consoleLog("Limelight: 8 corners not in network table");
            return ERROR_ANGLE;
        }

        // set up array prior to sorting xcorners and ycorners
        for (int i = 0; i < 8; i++)
        {
            pointArray[i] = new Point(xcorners[i], ycorners[i]);
        }

        // sort points into ascending x value order...
        Arrays.sort(pointArray, new pointCompare());

        double[] tempArrayx = new double[8];
        double[] tempArrayy = new double[8];

        for (int i = 0; i < 8; i++)
        {
            tempArrayx[i] = pointArray[i].x;
        }

        for (int i = 0; i < 8; i++)
        {
            tempArrayy[i] = pointArray[i].y;
        }

        SmartDashboard.putNumberArray("zzz sorted x points", tempArrayx);
        SmartDashboard.putNumberArray("zzz sorted y points", tempArrayy);

        // ...and stuff into imagePoints in following order: left bottom left, left top left, left bottom right, left top right,
        // right top left, right bottom left, right top right, right bottom right
        // Do this in ascending x order as we know contour corners will have lowest x point at bottom left (sorted earlier)
        MatOfPoint2f imagePoints = new MatOfPoint2f(pointArray[0], pointArray[1], /*pointArray[2],*/ pointArray[3], pointArray[4], /*pointArray[5],*/ pointArray[6], pointArray[7]);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        // solve for pose
        // rvec and tvec are 3x1
        try
        {
            Calib3d.solvePnP(m_objectPoints, imagePoints, m_cameraMatrix, m_distCoeffs, rvec, tvec);
        }
        catch (Throwable e)
        {
            // solvePnP failed in JNI cpp code -- log and return so robot code doesn't exit
            Logging.logException(e);
            return ERROR_ANGLE;
        }


        //Logging.consoleLog("rvec: " + rvec.dump());
        //Logging.consoleLog("tvec: " + tvec.dump());
		SmartDashboard.putString("zzz rvec", rvec.dump());
		SmartDashboard.putString("zzz rvec size", rvec.size().toString());
		SmartDashboard.putString("zzz tvec", tvec.dump());
        SmartDashboard.putString("zzz tvec size", tvec.size().toString());
        
        // using tvec, get distance to target as well as angle of robot to target.
        // angle should be close to that returned from x offset of Limelight.
        // tvec(0,0) is distance in x axis to target (left/right), and tvec(2,0) is z distance
        angleToTarget = computeAngle(tvec.get(0, 0)[0], tvec.get(2, 0)[0]);
        distance = MathHelper.getDistance(tvec.get(0, 0)[0], tvec.get(2, 0)[0]);
        SmartDashboard.putNumber("zzz angle to target", angleToTarget);
        SmartDashboard.putNumber("zzz distance", distance);

        try
        {
            // rotation matrix is 3x3.  Convert rotation vector into rotation matrix.
            Calib3d.Rodrigues(rvec, rotationMatrix);
        }
        catch (Throwable e)
        {
            // Rodrigues failed in JNI cpp code -- log and return so robot code doesn't exit
            Logging.logException(e);
            return ERROR_ANGLE;
        }

        // rotation of inverse.  We are trying to get the inverse transformation of the result from solvePnP.
        // Given transformation [R|t] (rvec), there is a shortcut to get the inverse transformation:
        // (inv[R|t]), which we are trying to obtain, is equal to (R'|-R'*t), where R' is R transposed.
        // rather than transpose with:
        // 
        // rotationMatrix = rotationMatrix.t();
        //
        // we tell gemm to do it with the GEMM_1_T flag.

        SmartDashboard.putString("zzz rotationMatrix", rotationMatrix.dump());
        SmartDashboard.putString("zzz rotationMatrix", rotationMatrix.size().toString());

        // translation of inverse
        // since in Java * operator doesn't work with Mat, this is
        // the equivalent of: pzero_world = -rotationMatrix.t() * tvec;
        Core.gemm(rotationMatrix, tvec, -1.0, new Mat(), 0.0, pzero_world, Core.GEMM_1_T);

        double double1 = pzero_world.get(0, 0)[0];
        double double2 = pzero_world.get(2, 0)[0];

        angleOfTarget = computeAngle(double1, double2);
        SmartDashboard.putNumber("zzz angle of target", angleOfTarget);


        /* experimental
        double horizontal = getTargetScreenWidth();
        double vertical = getTargetScreenHeight();
        double currentRatio = horizontal / vertical;

        // largest possible ratio from the front with no skew
        double maxRatio = 2.51;
        // acos of value > 1 results in NaN
        double ratio = Math.min(1, currentRatio / maxRatio);
        double simpleAngle = Math.toDegrees(Math.acos(ratio));
        SmartDashboard.putNumber("zzz simple angle from skew", simpleAngle);
        */

        // Check for validity:
        if(angleOfTarget > 90.0 || angleOfTarget < -90.0)
        {
            // The angle is out of the valid angle range,
            //return error.
            return ERROR_ANGLE;
        }

        return angleOfTarget;
    }

    public static double computeAngle(double x, double z)
    {
        double angle = Math.atan2(x, z) * (180 / Math.PI);
        return angle;
    }

    /**
     * Get the distance to a currently detected target.
     * Please ensure CAMERA_HEIGHT, CAMERA_ANGLE, and TARGET_HEIGHT are properly
     * set.
     * @return The distance, in inches.
     */
    public static double getDistance()
    {
        return getDistance(TARGET_HEIGHT, CAMERA_HEIGHT, CAMERA_ANGLE, getYOffset());
    }

    /**
     * Get the y-rotation of a target, given the target's width and screen width,
     * camera focal length, and distance.
     * @param targetWorldWidth The target's width in inches (world units).
     * @param targetScreenWidth The target's width in screen units (pixels).
     * @param distance The distance to the target.
     * @param focalLength The focal length of the camera.
     * 
     * @return The rotation of the object about the vertical axis.
     */
    public static double getYRotation(double targetWorldWidth, double targetScreenWidth, double distance,
            double focalLength)
    {
        // Get the bottom corners.
        MathHelper.Point2D topLeft = new MathHelper.Point2D(0, 0),
                           topRight = new MathHelper.Point2D(0, 0);

        boolean topLeftSuccess = getCorner(CornerTypes.TOP_LEFT, topLeft),
            topRightSuccess = getCorner(CornerTypes.TOP_RIGHT, topRight);

        if(topLeftSuccess && topRightSuccess)
        {   
            // Get the distance to each.
            double leftAngle = yAngleToPixel(topLeft),
                rightAngle = yAngleToPixel(topRight);

            // Get the distance to the left and right points.
            double distanceLeft = getDistance(TARGET_HEIGHT, CAMERA_HEIGHT, CAMERA_ANGLE, leftAngle),
                distanceRight = getDistance(TARGET_HEIGHT, CAMERA_HEIGHT, CAMERA_ANGLE, rightAngle);

            double deltaDistance = distanceLeft - distanceRight;

            double topWidth = TARGET_TOP_WIDTH;

            double result = Math.atan(deltaDistance / topWidth);

            result *= 180.0 / Math.PI; // Convert to degrees.
            
            return result;
        }

        return ERROR_ANGLE;
    }

    /**
     * Get the y rotation of the currently detected object.
     * Ensure the constant, TARGET_WIDTH, is set properly.
     * 
     * @return The rotation about the y-axis of the currently detected object.
     */
    public static double getYRotation()
    {
        initializeIfNeeded();

        return getYRotation(TARGET_WIDTH, getTargetScreenWidth(), getDistance(), FOCAL_LENGTH);
    }

    /**
     * Return the y-angle to a pixel on the screen. This is based
     * on the focal length, so the field of view of the camera.
     * See "5.4 From Pixels to Angles" in the Limelight documentation.
     * 
     * @param inputPoint The point to use. The x-component is ignored.
     * @return The y-angle to the provided point.
     */
    public static double yAngleToPixel(MathHelper.Point2D inputPoint)
    {
        double normalizedY = -(inputPoint.getY() - SCREEN_HEIGHT / 2);
        normalizedY /= SCREEN_HEIGHT / 2;

        double planeY = normalizedY * VIEW_PLANE_HEIGHT / 2.0;

        double angleToPixel = Math.atan2(planeY, FOCAL_LENGTH);

        angleToPixel *= 180.0 / Math.PI;

        return angleToPixel;
    }

    /**
     * Return the x-angle to a pixel on the screen.
     * See section 5.4 of the Limelight documentation, "From Pixels
     * to Angles."
     * 
     * @param inputPoint The point to use, the y-component is ignored.
     * @return The y-angle to the provided point.
     */
    public static double xAngleToPixel(MathHelper.Point2D inputPoint)
    {
        double normalizedX = -(inputPoint.getX() - SCREEN_WIDTH / 2.0);
        normalizedX /= SCREEN_WIDTH / 2.0;

        double planeX = normalizedX * VIEW_PLANE_WIDTH / 2.0;

        double angleToPixel = Math.atan2(planeX, FOCAL_LENGTH);

        return angleToPixel;
    }

    /**
     * Set the pipeline to be used by the limelight.
     * 
     * @param LimeLightPipeline The pipeline to be used.
     */
    public static void setPipeline(LimeLightPipeline pipeline)
    {
        initializeIfNeeded();

        // Get the pipeline's identifier.
        int pipeId = pipeline.getIdentifier();

        // Set this through network tables.
        setPipelineNetworkEntry.setNumber(pipeId);
    }

    /**
     * Get the index of the currently selected pipeline.
     * 
     * @return The identifier of the currently selected pipe.
     */
    public static int getSelectedPipelineId()
    {
        initializeIfNeeded();

        // Get the identifier of the pipeline.
        int pipeId = getPipelineNetworkEntry.getNumber(-1).intValue();

        // Return this value.
        return pipeId;
    }

    /**
     * Get a general property.
     * @param property
     * @return
     */
    public static double getGeneralProperty(AccessibleProperties property)
    {
        double result = 0.0;

        switch(property)
        {
            case HAS_TARGET:
                result = hasTarget() ? 1.0 : 0.0;
                break;
            case AREA:
                result = getTargetArea();
                break;
            case X_OFFSET:
                result = getXOffset();
                break;
            case Y_OFFSET:
                result = getYOffset();
                break;
            case DISTANCE:
                result = getDistance();
                break;
            case SKEW:
                result = getSkew();
                break;
            default:

                break;
        }

        return result;
    }

    /** 
     * Allows switching back and forth between the vision
     * processor and the limelight's driver camera mode.
    */
    public static void setCameraMode(boolean useProcessing)
    {
        initializeIfNeeded();

        if(useProcessing)
        {
            camMode.setNumber(0);
        }
        else
        {
            camMode.setNumber(1);
        }
    }

    public static void setLedMode(boolean bEnable)
    {
        initializeIfNeeded();

        // 0 uses current pipeline setting
        // 1 forces off
        // 2 forces blink
        // 3 forces on

        if (bEnable)
        {
            ledMode.setNumber(3);
        }
        else
        {
            ledMode.setNumber(1);
        }
    }

    /**
     * Enable/disable snapshot mode.
     * Enabling will take 2 snapshots per second and
     * store on the Limelight.
     */
    public static void setSnapshot(boolean bEnable)
    {
        initializeIfNeeded();

        if (bEnable)
        {
            snapshot.setNumber(1);
        }
        else
        {
            snapshot.setNumber(0);
        }
    }

    /**
     * Get the Limelight's reported latency.
     * Returns a value in milliseconds. According to
     * the limelight's documentation, at least 11 milliseconds
     * should be added for latency.
     */
    public static double getLatency()
    {
        initializeIfNeeded();

        return latency.getDouble(0.0);
    }

    /**
     * An outputter to the SmartDashboard for the LimeLight.
     */
    public static class LimeLightSmartDashboardOutputter implements DashboardOutputter
    {
        private SendableChooser<LimeLightPipeline> pipelineSelector;
        private LimeLightPipeline lastSelectedPipeline;
        private long lastUpdateTime;

        private static final long MINIMUM_UPDATE_TIME = 500;

        public void initializeDashboard()
        {
            // Set up values.
            SmartDashboard.setDefaultNumber("Distance (Trig)", -1.0);
            SmartDashboard.setDefaultNumber("Target Yaw", -1.0);
            SmartDashboard.setDefaultNumber("Target Yaw (Trig)", -1.0);
            SmartDashboard.setDefaultNumber("Camera Pitch", -1.0);
            SmartDashboard.setDefaultNumber("Distance for Pitch", 45.0);
            SmartDashboard.setDefaultString("Current Pipeline", "Not Initialized");

            // Set up the selector for the limelight's
            //pipeline IDs.
            pipelineSelector = new SendableChooser<LimeLightPipeline>();

            // Add configured pipelines to the selector.
            for(LimeLightPipeline pipe : LimeLightPipeline.values())
            {
                pipelineSelector.addOption(pipe.toString(), pipe);
            }

            // Set the default.
            pipelineSelector.setDefaultOption(DEFAULT_PIPELINE.toString(),
                DEFAULT_PIPELINE);
            
            // Note that the last selected pattern was the default.
            lastSelectedPipeline = DEFAULT_PIPELINE; // TODO: Ensure that this is true.

            // Store the last update time.
            lastUpdateTime = (new Date()).getTime();
        }

        public void updateDashboard()
        {
            double distanceForPitch = SmartDashboard.getNumber("Distance for Pitch", 45.0); // Retrieve the distance to be used for pitch from the SmartDashboard
                                                                                            // in case it has been changed by the user.
            long nowTime = (new Date()).getTime();
            long deltaTime = nowTime - lastUpdateTime;


            if (deltaTime > MINIMUM_UPDATE_TIME)
            {
                lastUpdateTime = (new Date()).getTime(); // Update the last calling time

                SmartDashboard.putNumber("Distance (Trig)", getDistance());
                SmartDashboard.putNumber("Target Yaw", angleOfTarget());
                SmartDashboard.putNumber("Target Yaw (Trig)", getYRotation());
                SmartDashboard.putNumber("Camera Pitch", calcCameraAngle(distanceForPitch));

                // Get pipeline information.
                LimeLightPipeline selectedPipeline = pipelineSelector.getSelected();
                LimeLightPipeline currentPipeline = LimeLightPipeline.getCurrentPipe();

                // Put the current pipeline on the dashboard, if it exists.
                if (currentPipeline != null)
                {
                    SmartDashboard.putString("Current Pipeline", currentPipeline.toString());
                }

                // If the pipeline has changed and isn't the current,
                if (selectedPipeline != lastSelectedPipeline
                        && selectedPipeline != currentPipeline)
                {
                    // Note that this pipeline was selected,
                    lastSelectedPipeline = selectedPipeline;

                    //and set it.
                    setPipeline(selectedPipeline);
                }
            }

        } // End updateDashboard.
    } // End LimeLightSmartDashboardOutputter.
}