package org.xero1425.base.swervedrive;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathManager;
import org.xero1425.misc.XeroPathSegment;

public class SwervePathFollowAction extends SwerveDriveAction {
    private int index_;
    private String pathname_;
    private XeroPath path_;
    private double[] angles_;
    private double[] speeds_;

    public SwervePathFollowAction(SwerveDriveSubsystem drive, String path) {

        super(drive);

        pathname_ = path;
        path_ = null;

        angles_ = new double[4] ;
        speeds_ = new double[4] ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        index_ = 0;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        if (index_ < path_.getSize())
        {
            XeroPathSegment fl = path_.getSegment(XeroPathManager.FL, index_) ;
            XeroPathSegment fr = path_.getSegment(XeroPathManager.FR, index_) ;
            XeroPathSegment bl = path_.getSegment(XeroPathManager.BL, index_) ;
            XeroPathSegment br = path_.getSegment(XeroPathManager.BR, index_) ;

            angles_[SwerveDriveSubsystem.FL] = fl.getHeading() ;
            angles_[SwerveDriveSubsystem.FR] = fr.getHeading() ;
            angles_[SwerveDriveSubsystem.BL] = bl.getHeading() ;
            angles_[SwerveDriveSubsystem.BR] = br.getHeading() ;

            speeds_[SwerveDriveSubsystem.FL] = fl.getVelocity() ;
            speeds_[SwerveDriveSubsystem.FR] = fr.getVelocity() ;
            speeds_[SwerveDriveSubsystem.BL] = bl.getVelocity() ;
            speeds_[SwerveDriveSubsystem.BR] = br.getVelocity() ;

            getSubsystem().setTargets(angles_, speeds_);

            index_++ ;
        }

        if (index_ == path_.getSize())
        {
            getSubsystem().stop();
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        setDone();
    }

    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveDriveFollowPathAction-" + pathname_ ;
        return ret ;
    }
}