package org.xero1425.base.tankdrive;

public class TankDriveVelocities {
    private double left_ ;
    private double right_ ;

    public TankDriveVelocities(double l, double r) {
        left_ = l ;
        right_ = r ;
    }

    double getLeft() {
        return left_ ;
    }

    double getRight() {
        return right_ ;
    }
}
