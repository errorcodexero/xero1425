package org.xero1425.base.tankdrive;

/// \file

/// \brief This class stores velocities for the left and right wheels of a tank drive
public class TankDriveVelocities {
    private double left_ ;
    private double right_ ;

    public TankDriveVelocities(double l, double r) {
        left_ = l ;
        right_ = r ;
    }

    public double getLeft() {
        return left_ ;
    }

    public double getRight() {
        return right_ ;
    }
}
