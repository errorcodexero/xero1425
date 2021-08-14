package org.xero1425.base.swervedrive;

import org.xero1425.base.Subsystem.DisplayType;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroMath;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class SwerveRamseteAction extends SwerveDriveAction {
    private String pathname_ ;
    private XeroPath path_ ;
    private double b_ ;
    private double zeta_ ;
    private RamseteController ctrl_ ;
    private int index_ = 0 ;

    private final int MainRobot = 0;

    public SwerveRamseteAction(SwerveDriveSubsystem sub, String pathname) {
        this(sub, pathname, 2.1, 0.8) ;
    }

    public SwerveRamseteAction(SwerveDriveSubsystem sub, String pathname, double b, double zeta) {
        super(sub) ;

        b_ = b ;
        zeta_ = zeta ;
        pathname_ = pathname ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        getSubsystem().startSwervePlot();

        ctrl_ = new RamseteController(b_, zeta_) ;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);
        index_ = 0 ;

        XeroPathSegment seg = path_.getSegment(MainRobot, 0) ;
        Pose2d start = new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
        getSubsystem().startOdometry(start) ;
    }

    @Override
    public void run() {
        if (index_ < path_.getSize()) {
            SwerveDriveSubsystem sub = getSubsystem() ;

            // Current pose in inches
            Pose2d currentPoseInches = sub.getPose() ;
            Pose2d currentPose = new Pose2d(XeroMath.inchesToMeters(currentPoseInches.getX()), XeroMath.inchesToMeters(currentPoseInches.getY()), currentPoseInches.getRotation()) ;

            sub.putDashboard("db-trk-t", DisplayType.Always, sub.getRobot().getTime()) ;
            sub.putDashboard("db-trk-x", DisplayType.Always, currentPoseInches.getX()) ;
            sub.putDashboard("db-trk-y", DisplayType.Always, currentPoseInches.getY()) ;
            sub.putDashboard("db-trk-a", DisplayType.Always, currentPoseInches.getRotation().getDegrees()) ;

            // Current segment in inches
            XeroPathSegment seg = path_.getSegment(MainRobot, index_) ;

            sub.putDashboard("db-path-t", DisplayType.Always, sub.getRobot().getTime()) ;
            sub.putDashboard("db-path-x", DisplayType.Always, seg.getX()) ;
            sub.putDashboard("db-path-y", DisplayType.Always, seg.getY()) ;
            sub.putDashboard("db-path-a", DisplayType.Always, seg.getHeading()) ;

            // Desired pose in meters
            Pose2d desiredPose = new Pose2d(XeroMath.inchesToMeters(seg.getX()), 
                                                XeroMath.inchesToMeters(seg.getY()), 
                                                Rotation2d.fromDegrees(seg.getHeading())) ;

            // The desired linear velocity in meters
            double linearVelocityRefMeters = XeroMath.inchesToMeters(seg.getVelocity()) ;

            // The desired angular velocity in radians per second
            double angularVelocityRefRadiansPerSecond = 0.0 ;

            if (index_ != 0) {
                XeroPathSegment prev = path_.getSegment(MainRobot, index_ - 1) ;

                // Compute the angular velocity in radians per second
                angularVelocityRefRadiansPerSecond = XeroMath.normalizeAngleDegrees(seg.getHeading() - prev.getHeading()) / getSubsystem().getRobot().getPeriod() / 180.0 * Math.PI ;
            }

            // Robot speed in meters per second
            ChassisSpeeds speeds = ctrl_.calculate(currentPose, desiredPose, linearVelocityRefMeters, angularVelocityRefRadiansPerSecond) ;
            MessageLogger logger = sub.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub.getLoggerID()) ;
            logger.add("vx", speeds.vxMetersPerSecond) ;
            logger.add("vy", speeds.vyMetersPerSecond) ;
            logger.add("ang", XeroMath.rad2deg(speeds.omegaRadiansPerSecond)) ;
            logger.add("pose-angle", currentPose.getRotation().getDegrees()) ;
            logger.add("target-angle", desiredPose.getRotation().getDegrees()) ;
            logger.endMessage(); 

            getSubsystem().setChassisSpeeds(speeds) ;

            index_++ ;
        }

       if (index_ == path_.getSize())
       {
            getSubsystem().endSwervePlot();
            try {
                getSubsystem().stop() ;
            }
            catch(Exception ex) {
            }
           setDone() ;
       }            
    }

    @Override
    public void cancel() {
    }


    @Override
    public String toString(int ident) {
        return "SwerveRamseteAction-" + pathname_ ;
    }
}
