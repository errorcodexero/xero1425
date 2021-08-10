package org.xero1425.base.swervedrive;

import org.graalvm.compiler.phases.common.FloatingReadPhase;
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
import org.xero1425.misc.XeroMath;

//import edu.wpi.first.wpilibj.PIDController;   //don't use this...
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {

    private XeroRobot robot_ ;
    private String name_ ;

    private MotorController steer_ ;
    private MotorController drive_ ;
    private XeroEncoder encoder_ ;

    private double steer_power_ ;
    private double drive_power_ ;

    private boolean has_steer_target_ ;
    private boolean has_drive_target_ ;

    private double target_angle_ ;
    private double target_speed_ ;

    private PIDCtrl angle_pid_ ;
    private PIDCtrl speed_pid_ ;

    private Speedometer linear_ ;
    private double ticks_ ;
    private double inches_per_tick_ ;
    static private int logger_id_ = -1 ;

    private final String LinearSamplesName = "swervedrive:linear:samples" ;
    private final String InchesPerTickName = "swervedrive:inches_per_tick" ;

    public SwerveModule(XeroRobot robot, SwerveDriveSubsystem subsystem, String name, String config) 
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException, BadMotorRequestException {
        
        robot_ = robot ;
        name_ = name ;

        SettingsParser settings = subsystem.getRobot().getSettingsParser() ;
        MessageLogger logger = subsystem.getRobot().getMessageLogger() ;

        inches_per_tick_ = settings.get(InchesPerTickName).getDouble() ;

        steer_ = robot.getMotorFactory().createMotor(name + "-steer", config + ":steer") ;
        drive_ = robot.getMotorFactory().createMotor(name + "-drive", config + ":drive") ;
        encoder_ = new XeroEncoder(robot, config + ":encoder", true, null) ;

        steer_power_ = 0 ;   //is a double
        drive_power_ = 0 ;   //is a double
        
        has_steer_target_ = false ;
        has_drive_target_ = false ;

        target_angle_ = 0 ;  //is a double
        target_speed_ = 0 ;  //is a double

        angle_pid_ = new PIDCtrl(robot.getSettingsParser(), config + ":steer:pid", true) ; //replace "robot.getSettingsParser" with "settings" ??
        speed_pid_ = new PIDCtrl(robot.getSettingsParser(), config + ":drive:pid", false) ;

        int samples = 2 ;
        if (settings.isDefined(LinearSamplesName) && settings.get(LinearSamplesName).isInteger()) {
            samples = settings.get(LinearSamplesName).getInteger() ;
        }

        linear_ = new Speedometer("linear", samples, false) ;

        if (logger_id_ == -1) {
            logger_id_ = logger.registerSubsystem("swervemodule") ;
        }

    }

    public void run(double dt) throws BadMotorRequestException, MotorRequestFailedException {

        MessageLogger logger = robot_.getMessageLogger() ;
        logger.startMessage(MessageType.Debug, logger_id_) ;
        logger.add(name_) ;

        if(has_steer_target_) {
            double out = angle_pid_.getOutput(target_angle_, getAngle(), dt) ;

            logger.add("[AnglePID") ;
            logger.add("target", target_angle_) ;
            logger.add("actual", getAngle()) ;
            logger.add("PIDout", out) ;
            logger.add("]") ;
            steer_.set(out) ;
        }

        if(has_drive_target_) {
            double out = speed_pid_.getOutput(target_speed_, getSpeed(), dt) ;

            logger.add("[SpeedPID") ;
            logger.add("target", target_angle_) ;
            logger.add("actual", getSpeed()) ;
            logger.add("PIDout", out) ;
            logger.add("]") ;
            steer_.set(out) ;
        }
        logger.endMessage();
    }

    public void computeMyState(double dt) throws BadMotorRequestException {
        ticks_ = drive_.getPosition() ;
        linear_.update(dt, ticks_ * inches_per_tick_) ;
    }

    public SwerveModuleState getModuleStateMeters() {  //Why use meter conversion -- we usually do inches...
        return new SwerveModuleState(XeroMath.inchesToMeters(getSpeed()), Rotation2d.fromDegrees(getAngle()))
    }

    public double getAcceleration() {
        return linear_.getAcceleration() ;
    }

    //consider changing all "get speed" into "get velocity" bc we want the direction
    //  in addition to magnitude...
    public double getSpeed() {   
        return linear_.getVelocity() ;
        
    }

    public double getDistance() {
        return linear_.getDistance() ;
    }

    public double getSpeedTarget() {
        return target_speed_ ;
    }

    public double getAngle() {
        return encoder_.getPosition() ;
    }

    public double getAngleTarget() {
        return target_angle_ ;
    }

    public void setNeutralMode(MotorController.NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        steer_.setNeutralMode(mode) ;
        drive_.setNeutralMode(mode) ;
    }

    public double steerPower() {
        return steer_power_ ;
    }

    public double drivePower() {
        return drive_power_ ;
    }

    public void setSteerMotorPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        has_steer_target_ = false ;
        steer_power_ = p ;
        steer_.set(p) ;
    }

    public void setDriveMotorPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        has_drive_target_ = false ;
        drive_power_ = p ;
        drive_.set(p) ;
    }

    public void setPower(double steer, double drive) throws BadMotorRequestException, MotorRequestFailedException {
        setSteerMotorPower(steer) ;
        setDriveMotorPower(drive) ;
    }

    public double getTicks() {
        return ticks_ ;
    }

    public void setTargets(double angle, double speed) {
        
        double dist = Math.abs(XeroMath.normalizeAngleDegrees(angle - getAngle()))
        if(Math.abs(dist) > 90.0) {
            angle = XeroMath.normalizeAngleDegrees(angle + 180) ;
            speed = -speed ; 
        }

        target_angle_ = angle ;
        target_speed_ = speed ;

        //moved to end of function as opposed to at the beginning
        has_steer_target_ = true ;
        has_drive_target_ = true ;
    }

    public void setAngle(double angle) {
        target_angle_ = angle ;
        //again, moved to end of function as opposed to at the beginning
        has_steer_target_ = true ;
    }

    public String status() {
        String str = String.format("%.2f @ %.2f", getSpeed(), getAngle()) ;

        if(has_steer_target_ && has_drive_target_) {
            str += " t: " + String.format("%.2f @ %.2f", target_speed_, target_angle_) ;
        }
        
        return str ;
    }


}
