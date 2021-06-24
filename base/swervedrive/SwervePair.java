package org.xero1425.base.swervedrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.EncoderConfigException;
import org.xero1425.base.motorsubsystem.XeroEncoder;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class SwervePair {
    private MotorController steer_;
    private MotorController drive_;
    private XeroEncoder encoder_;
    private double steer_power_;
    private double drive_power_;

    public SwervePair(XeroRobot robot, String name, String config) throws BadParameterTypeException,
            MissingParameterException, EncoderConfigException, BadMotorRequestException {
        steer_ = robot.getMotorFactory().createMotor(name + "-steer", config + ":steer");
        drive_ = robot.getMotorFactory().createMotor(name + "-drive", config + ":drive");
        encoder_ = new XeroEncoder(robot, config + ":encoder", true, null) ;

        drive_power_ = 0.0;
        steer_power_ = 0.0;
    }

    public double angle() {
        return encoder_.getPosition() ;
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
