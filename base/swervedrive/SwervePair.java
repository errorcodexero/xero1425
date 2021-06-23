package org.xero1425.base.swervedrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;

public class SwervePair {
    private MotorController steer_;
    private MotorController drive_;
    private double steer_power_;
    private double drive_power_;

    //
    // name e.g. - hw:swervedrive:motors:fl
    public SwervePair(XeroRobot robot, String name, String config) {
        steer_ = robot.getMotorFactory().createMotor(name + "-steer", config + ":steer");
        drive_ = robot.getMotorFactory().createMotor(name + "-drive", config + ":drive");

        drive_power_ = 0.0;
        steer_power_ = 0.0;
    }

    public void setNeutralMode(MotorController.NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        steer_.setNeutralMode(mode);
        drive_.setNeutralMode(mode);
    }

    public double steerPower() {
        return steer_power_;
    }

    public double drivePower() {
        return drive_power_;
    }

    public void setSteerPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        steer_power_ = p ;
        steer_.set(p) ;
    }

    public void setDrivePower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        drive_power_ = p ;
        drive_.set(p);
    }

    public void set(double steer, double drive) throws BadMotorRequestException, MotorRequestFailedException {
        setSteerPower(steer);
        setDrivePower(drive) ;
    }
}
