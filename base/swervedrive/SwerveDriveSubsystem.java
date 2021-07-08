package org.xero1425.base.swervedrive;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.SwerveDriveGamepad;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.SettingsParser;

public class SwerveDriveSubsystem extends DriveBaseSubsystem {
    private double width_;
    private double length_;
    private SwerveModule[] pairs_;

    static public final int FL = 0;
    static public final int FR = 1;
    static public final int BL = 2;
    static public final int BR = 3;
    static private final int LAST_MODULE = 3;

    static private String[] cname = { "fl", "fr", "bl", "br" };
    static private String[] hname = { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };

    public SwerveDriveSubsystem(Subsystem parent, String name, String config) throws Exception {
        super(parent, name);

        if (cname.length != hname.length) {
            throw new Exception("Invalid swerve drive code, cname and hname must be the same length");
        }

        SettingsParser settings = getRobot().getSettingsParser();

        width_ = settings.get("swervedrive:width").getDouble();
        length_ = settings.get("swervedrive:length").getDouble();

        pairs_ = new SwerveModule[cname.length];
        for (int i = 0; i < cname.length; i++) {
            pairs_[i] = new SwerveModule(getRobot(), this, hname[i], config + ":" + cname[i]);
        }
    }

    public int getModuleCount() {
        return pairs_.length ;
    }

    public double getWidth() {
        return width_;
    }

    public double getLength() {
        return length_;
    }

    public double getAcceleration() {
        double total = 0.0 ;

        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            total += pairs_[i].getAcceleration() ;
        }

        return total / pairs_.length ;
    }

    public double getVelocity() {
        double total = 0.0 ;

        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            total += pairs_[i].getVelocity() ;
        }

        return total / pairs_.length ;
    }

    public double getDistance() {
        double total = 0.0 ;

        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            total += pairs_[i].getDistance() ;
        }

        return total / pairs_.length ;
    }

    public Gamepad createGamePad(OISubsystem oi, int index, DriveBaseSubsystem drive) throws Exception {
        return new SwerveDriveGamepad(oi, index, drive);
    }

    /// \brief returns true to indicate this is a drivebase
    /// \returns true to indicate this is a drivebase
    public boolean isDB() {
        return true;
    }

    /// \brief This method is called when the robot enters one of its specifc modes.
    /// The modes are Autonomous, Teleop, Test, or Disabled. It is used to set the
    /// neutral mode specifically for the robot mode.
    public void init(LoopType ltype) {
        super.init(ltype);

        MotorController.NeutralMode nm = loopTypeToNeutralMode(ltype);
        for (int i = 0; i < cname.length; i++) {
            try {
                pairs_[i].setNeutralMode(nm);
            } catch (Exception ex) {
            }
        }
    }

    public String getPairName(int which, boolean sname) {
        String ret = "????";

        if (which >= 0 && which < cname.length) {
            if (sname)
                ret = cname[which];
            else
                ret = hname[which];
        }

        return ret;
    }

    public void computeMyState() throws BadMotorRequestException {
        MessageLogger logger = getRobot().getMessageLogger();

        for (int i = 0; i < pairs_.length; i++)
            pairs_[i].computeMyState(getRobot().getDeltaTime());

        putDashboard("flangle", DisplayType.Always, pairs_[FL].angle());
        putDashboard("frangle", DisplayType.Always, pairs_[FR].angle());
        putDashboard("blangle", DisplayType.Always, pairs_[BL].angle());
        putDashboard("brangle", DisplayType.Always, pairs_[BR].angle());
    }

    @Override
    public void run() throws Exception {
        super.run();

        double dt = getRobot().getDeltaTime();
        for (int i = 0; i < pairs_.length; i++)
            pairs_[i].run(dt);

    }

    public void stop() throws BadMotorRequestException, MotorRequestFailedException {
        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            pairs_[i].set(0.0, 0.0) ;
        }
    }

    public void setPower(int which, double steer, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        pairs_[which].set(steer, drive) ;
    }

    
    public void setSteer(int which, double steer) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        pairs_[which].setSteerPower(steer) ;
    }

    
    public void setDrive(int which, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        pairs_[which].setDrivePower(drive) ;
    }

    public void setTargets(double[] angles, double[] speeds) {
        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            pairs_[i].setTargetAngle(angles[i]);
            pairs_[i].setTargetVelocity(speeds[i]) ;
        }
    }

    public void setAngle(double angle) {
        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            pairs_[i].setTargetAngle(angle);
        }        
    }

    public void setNoTargets() {
        for(int i = 0 ; i <= LAST_MODULE ; i++) {
            pairs_[i].setNoAngle();
            pairs_[i].setNoSpeed();
        }
    }
}
