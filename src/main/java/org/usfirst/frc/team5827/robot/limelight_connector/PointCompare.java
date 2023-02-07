package org.usfirst.frc.team5827.robot.limelight_connector;

import java.util.Comparator;
import org.opencv.core.Point;

class pointCompare implements Comparator<Point>
{
    public int compare(Point a, Point b)
    {
        return Double.compare(a.x, b.x);
        // This would do the same thing, but why not use existing Double.compare...
        // return (a.x < b.x) ? -1 : (a.x > b.x) ? 1 : 0;
    }
}