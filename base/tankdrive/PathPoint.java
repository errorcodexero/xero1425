package org.xero1425.base.tankdrive;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/// \brief describes a point along a path that might lie between two segments
public class PathPoint {
    //
    // The first of the two segments that contain the point
    //
    private int which_ ;

    //
    // The percentage the point is between the two segments
    //
    private double pcnt_ ;

    //
    // The location of the point
    //
    private Translation2d loc_ ;

    public PathPoint(int which, double pcnt, Translation2d loc) {
        which_ = which ;
        pcnt_ = pcnt ;
        loc_ = loc ;
    }

    public int which() {
        return which_ ;
    }

    public double percent() {
        return pcnt_ ;
    }

    public Translation2d loc() {
        return loc_ ;
    }
}
