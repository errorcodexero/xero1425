package org.xero1425.base.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavxGyro implements XeroGyro {
    private AHRS navx_ ;

    public NavxGyro() {
        navx_ = new AHRS(SPI.Port.kMXP) ;
    }

    public boolean isConnected() {
        return navx_.isConnected() ;
    }

    public double getYaw() {
        return -navx_.getYaw() ;
    }

    public double getAngle() {
        return navx_.getAngle() ;
    }

    public void reset() {
        navx_.reset() ;
    }
}
