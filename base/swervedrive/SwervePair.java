package org.xero1425.base.swervedrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.EncoderConfigException;
import org.xero1425.base.motorsubsystem.XeroEncoder;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

public class SwervePair {
    private MotorController steer_;
    private MotorController drive_;
    private XeroEncoder encoder_;
    private double steer_power_;
    private double drive_power_;
    private boolean has_angle_target_ ;
    private double target_angle_ ;
    private boolean has_speed_target_ ;
    private double target_speed_ ;
    private PIDCtrl angle_pid_ ;
    private PIDCtrl speed_pid_ ;

    public SwervePair(XeroRobot robot, String name, String config) throws BadParameterTypeException,
            MissingParameterException, EncoderConfigException, BadMotorRequestException {
        steer_ = robot.getMotorFactory().createMotor(name + "-steer", config + ":steer");
        drive_ = robot.getMotorFactory().createMotor(name + "-drive", config + ":drive");
        encoder_ = new XeroEncoder(robot, config + ":encoder", true, null) ;

        drive_power_ = 0.0;
        steer_power_ = 0.0;

        has_angle_target_ = false ;
        has_speed_target_ = false ;
        target_angle_ = 0.0 ;
        target_speed_ = 0.0 ;

        angle_pid_ = new PIDCtrl(robot.getSettingsParser(), config + ":steer:pid", true) ;
        speed_pid_ = new PIDCtrl(robot.getSettingsParser(), config + ":speed:pid", false) ;
    }

    public void run(double dt) throws BadMotorRequestException, MotorRequestFailedException {
        if (has_angle_target_)
        {
            double out = angle_pid_.getOutput(target_angle_, angle(), dt) ;
            steer_.set(out) ;
        }

        if (has_speed_target_)
        {
            double out = speed_pid_.getOutput(target_speed_, speed(), dt) ;
            drive_.set(out) ;
        }
    }

    public double speed() {
        return 0.0 ;
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

    public void setTargetAngle(double angle) {
        has_angle_target_ = true ;
        target_angle_ = angle ;
    }

    public void setNoAngle() {
        has_angle_target_ = false ;
    }

    public void setTargetSpeed(double speed) {
        has_speed_target_ = true ;
        target_speed_ = speed ;
    }

    public void setNoSpeed() {
        has_speed_target_ = false ;
    }
}
