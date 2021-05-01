package org.xero1425.base.tankdrive;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class LookAheadPoint {
    private Pose2d pose_ ;
    private boolean atend_ ;

    public LookAheadPoint(Pose2d pose, boolean end) {
        pose_ = pose ;
        atend_ = end ;
    }

    public Pose2d getPose() {
        return pose_ ;
    }

    public boolean atEnd() {
        return atend_ ;
    }
}
