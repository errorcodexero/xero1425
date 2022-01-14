package org.xero1425.base.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetTracker {
    
    public TargetTracker(Pose2d goal) throws Exception {
        // initialize goal to smth...
        // take in the turret's offset in x and y?

    }

    public double getAngleDifference(Pose2d robot) {
        //aka, gets the relative [to robot] angle target
        // perhaps rename this method...

        //TODO: 
        // perform calculations 
        // https://docs.google.com/document/d/1I46HP8EBB1ETDHYZUqWiXNq7MdrrvPTCe26EkCgO5Uc/edit

        // returns T theta

        return 0.0 ;
    }

    public double getDistance(Pose2d robot) {
        // perhaps rename this method...

        // make calculations based on Pythagorean's theorem; goal; and robot

        // returns distance

        return 0.0 ;
    }

}
