package org.xero1425.base.swervedrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.EncoderConfigException;
import org.xero1425.base.motorsubsystem.XeroEncoder;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;
import org.xero1425.misc.SettingsParser;
import org.xero1425.misc.Speedometer;

public class SwerveModule {
    private XeroRobot robot_ ;
    private SwerveDriveSubsystem subsystem_ ;
    private String name_ ;
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
    private Speedometer linear_ ;
    private double inches_per_tick_ ;

    private final String LinearSamplesName = "swervedrive:linear:samples" ;
    private final String InchesPerTickName = "swervedrive:inches_per_tick" ;

    public SwerveModule(XeroRobot robot, SwerveDriveSubsystem subsystem, String name, String config) throws BadParameterTypeException,
            MissingParameterException, EncoderConfigException, BadMotorRequestException {

        SettingsParser settings = subsystem.getRobot().getSettingsParser() ;

        inches_per_tick_ = settings.get(InchesPerTickName).getDouble() ;

        robot_ = robot ;
        subsystem_ = subsystem ;
        name_ = name ;

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

        int samples =  2 ;
        if (settings.isDefined(LinearSamplesName) && settings.get(LinearSamplesName).isInteger())
            samples = settings.get(LinearSamplesName).getInteger() ;
            
        linear_ = new Speedometer("linear", samples, false) ;
    }

    public void run(double dt) throws BadMotorRequestException, MotorRequestFailedException {

        MessageLogger logger = robot_.getMessageLogger() ;
        logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
        logger.add(name_) ;

                
        if (has_angle_target_)
        {
            double out = angle_pid_.getOutput(target_angle_, angle(), dt) ;

            logger.add(" AnglePID") ;
            logger.add("target", target_angle_) ;
            logger.add("angle", angle()) ;
            logger.add("out", out) ;
            steer_.set(out) ;
        }

        if (has_speed_target_)
        {
            double out = speed_pid_.getOutput(target_speed_, getVelocity(), dt) ;
            logger.add(" SpeedPID") ;
            logger.add("target", target_speed_) ;
            logger.add("speed", getVelocity()) ;
            logger.add("out", out) ;
            drive_.set(out) ;
        }
        logger.endMessage();
    }

    public void computeMyState(double dt) throws BadMotorRequestException {
        double dist = drive_.getPosition() * inches_per_tick_ ;
        linear_.update(dt, dist);
    }

    public double getAcceleration() {
        return linear_.getAcceleration() ;
    }

    public double getVelocity() {
        return linear_.getVelocity() ;
    }

    public double getDistance() {
        return linear_.getDistance() ;
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

    public void setTargetVelocity(double speed) {
        has_speed_target_ = true ;
        target_speed_ = speed ;
    }

    public void setNoSpeed() {
        has_speed_target_ = false ;
    }
}
