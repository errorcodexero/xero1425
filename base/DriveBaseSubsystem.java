package org.xero1425.base;

import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.RomiGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.motors.MotorController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.XeroMath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public abstract class DriveBaseSubsystem extends Subsystem {
    private XeroGyro gyro_;
    private MotorController.NeutralMode automode_neutral_;
    private MotorController.NeutralMode teleop_neutral_;
    private MotorController.NeutralMode disabled_neutral_;

    public DriveBaseSubsystem(Subsystem parent, String name)
            throws BadParameterTypeException, MissingParameterException {
        super(parent, name);

        MessageLogger logger = getRobot().getMessageLogger();

        automode_neutral_ = MotorController.NeutralMode.Brake;
        teleop_neutral_ = MotorController.NeutralMode.Brake;
        disabled_neutral_ = MotorController.NeutralMode.Coast;

        String gyrotype = getRobot().getSettingsParser().get("hw:gyro").getString();
        if (gyrotype.equals("navx")) {
            gyro_ = new NavxGyro();
        } else if (gyrotype.equals("LSM6DS33")) {
            gyro_ = new RomiGyro();
        }

        double start = getRobot().getTime();
        while (getRobot().getTime() - start < 3.0) {
            if (gyro().isConnected())
                break;
        }

        if (!gyro_.isConnected()) {
            logger.startMessage(MessageType.Error);
            logger.add("NavX is not connected - cannot perform tankdrive path following functions");
            logger.endMessage();
            gyro_ = null;
        }
    }

    public static Pose2d inchesToMeters(Pose2d p) {
        return new Pose2d(XeroMath.inchesToMeters(p.getX()), XeroMath.inchesToMeters(p.getY()), p.getRotation()) ;
    }

    public static Pose2d metersToInches(Pose2d p) {
        return new Pose2d(XeroMath.metersToInches(p.getX()), XeroMath.metersToInches(p.getY()), p.getRotation()) ;
    }

    public static Pose2d segmentToPose(XeroPathSegment seg) {
        return new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
    }

    /// \brief returns true to indicate this is a drivebase
    /// \returns true to indicate this is a drivebase
    public boolean isDB() {
        return true;
    }

    protected boolean hasGyro() {
        return gyro_ != null ;
    }

    protected XeroGyro gyro() {
        return gyro_ ;
    }

    protected MotorController.NeutralMode autoModeNeutral() {
        return automode_neutral_ ;
    }

    protected MotorController.NeutralMode teleopModeNeutral() {
        return teleop_neutral_ ;
    }

    protected MotorController.NeutralMode disabledModeNeutral() {
        return disabled_neutral_ ;
    }

    protected MotorController.NeutralMode loopTypeToNeutralMode(LoopType ltype) {
        MotorController.NeutralMode nm = MotorController.NeutralMode.Brake;
        
        switch (ltype) {
            case Autonomous:
                nm = automode_neutral_ ;
                break;

            case Teleop:
                nm = teleop_neutral_ ;
                break;

            case Test:
                nm = disabled_neutral_ ;
                break;

            case Disabled:
                nm = disabled_neutral_ ;           
                break ;
        }

        return nm ;
    }
}
