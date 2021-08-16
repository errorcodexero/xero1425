package org.xero1425.base.swervedrive;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.Subsystem.DisplayType;
import org.xero1425.base.utils.LineSegment;
import org.xero1425.base.utils.LookAheadPoint;
import org.xero1425.base.utils.PathPoint;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.XeroMath;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwervePurePursuit extends SwerveDriveAction {
    private String pathname_;
    private XeroPath path_;
    private double[] angles_;
    private double[] speeds_;
    private double look_ahead_distance_;

    static private final int MainRobot = 0;

    public SwervePurePursuit(SwerveDriveSubsystem sub, String pathname)
            throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        pathname_ = pathname ;
        angles_ = new double[getSubsystem().getModuleCount()] ;
        speeds_ = new double[getSubsystem().getModuleCount()] ;
        
        look_ahead_distance_ = sub.getRobot().getSettingsParser().get("swervedrive:lookahead").getDouble() ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        getSubsystem().startSwervePlot();

        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);
        Pose2d initial = DriveBaseSubsystem.segmentToPose(path_.getSegment(MainRobot, 0)) ;
        getSubsystem().resetOdometry(initial) ;
    }

    @Override
    public void run() {
        //
        // Get the subsystem
        //
        SwerveDriveSubsystem sub = getSubsystem() ;

        //
        // Get the current subsystem pose
        //
        Pose2d pos = getSubsystem().getPose() ;
            
        //
        // Find the point on the path that is closest in distance to the robots
        // current position
        //
        PathPoint closest = findClosestPoint(pos) ;

        //
        // Find point at the look ahead distance from here to the robot
        //
        LookAheadPoint look = findLookAheadPoint(closest) ;

        getSubsystem().putDashboard("db-path-t", DisplayType.Verbose, getSubsystem().getRobot().getTime()) ;
        getSubsystem().putDashboard("db-path-x", DisplayType.Verbose, look.getPose().getX()) ;
        getSubsystem().putDashboard("db-path-y", DisplayType.Verbose, look.getPose().getY()) ;
        getSubsystem().putDashboard("db-path-a", DisplayType.Verbose, look.getPose().getRotation().getDegrees()) ;

        sub.putDashboard("db-trk-t", DisplayType.Always, sub.getRobot().getTime()) ;
        sub.putDashboard("db-trk-x", DisplayType.Always, pos.getX()) ;
        sub.putDashboard("db-trk-y", DisplayType.Always, pos.getY()) ;
        sub.putDashboard("db-trk-a", DisplayType.Always, pos.getRotation().getDegrees()) ;

        if (!look.atEnd())
        {
            Pose2d correction = look.getPose().relativeTo(pos) ;

            double velocity ;
            int which = closest.which() ;
            if (which == path_.getSize() - 2)
            {
                //
                // We are on the last segment
                //
                velocity = path_.getSegment(MainRobot, which).getVelocity() ;
            }
            else
            {
                velocity = XeroMath.interpolate(closest.percent(), 
                    path_.getSegment(MainRobot, which + 1).getVelocity(), 
                    path_.getSegment(MainRobot, which + 2).getVelocity()) ;
            }

            double angle = Math.atan2(correction.getY(), correction.getX()) ;

            for(int i = 0 ; i < sub.getModuleCount() ; i++)
            {
                angles_[i] = angle ;
                speeds_[i] = velocity ;
                sub.setTargets(angles_, speeds_);
            }
        }
        else
        {
            sub.endSwervePlot();
            try {
                sub.stop() ;
            }
            catch(Exception ex) {

            }
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        try {
            getSubsystem().stop() ;
        }
        catch(Exception ex) {
        }

        setDone() ;
    }

    @Override
    public String toString(int ident) {
        return "SwervePurePursuit-" + pathname_ ;
    }

    private PathPoint findClosestPoint(Pose2d pos) {
        double dist = Double.MAX_VALUE ;
        int which = -1 ;
        double pcnt = 0.0 ;

        Translation2d p1 = null, p2 = null ;

        for(int i = 0 ; i < path_.getSize() - 1; i++) {
            XeroPathSegment seg0 = path_.getSegment(MainRobot, i) ;
            XeroPathSegment seg1 = path_.getSegment(MainRobot, i + 1) ;

            LineSegment ls = new LineSegment(seg0.getX(), seg0.getY(), seg1.getX(), seg1.getY()) ;
            Translation2d closest = ls.closest(pos.getTranslation()) ;
            double clpcnt = ls.dotProd(pos.getTranslation()) / ls.length() ;

            double ptdist = closest.getDistance(pos.getTranslation()) ;
            if (ptdist < dist)
            {
                which = i ;
                dist = ptdist ;
                pcnt = clpcnt ;
                p1 = new Translation2d(seg0.getX(), seg0.getY()) ;
                p2 = new Translation2d(seg1.getX(), seg1.getY()) ;
            }
        }

        double x = p1.getX() + (p2.getX() - p2.getX()) * pcnt ;
        double y = p1.getY() + (p2.getY() - p2.getY()) * pcnt ;
        return new PathPoint(which, pcnt, new Translation2d(x, y)) ;
    }

    private LookAheadPoint findLookAheadPoint(PathPoint pt) {
        LookAheadPoint ret = null ;

        double remaining = look_ahead_distance_ ;

        for(int i = pt.which() ; i < path_.getSize() - 1 ; i++)
        {
            XeroPathSegment seg0 = path_.getSegment(MainRobot, i);
            double prev = 0.0 ;
            if (i != 0)
                prev = path_.getSegment(MainRobot, i - 1).getPosition() ;

            double segsize = seg0.getPosition() - prev ;

            if (remaining < segsize)
            {
                //
                // The look ahead point is in this segment
                //

                XeroPathSegment seg1 = path_.getSegment(MainRobot, i + 1) ;

                // Calculate how far is the look ahead point between this segment and the next
                double pcnt = 1 - remaining / segsize ;
                ret = new LookAheadPoint(interpolate(pcnt, segmentToPose(seg0), segmentToPose(seg1)), false) ;
                break ;
            }

            remaining -= segsize ;
        }

        if (ret == null)
        {
            //
            // The look ahead point exceeds the path
            //
            XeroPathSegment seg = path_.getSegment(MainRobot, path_.getSize() - 1) ;
            Pose2d endpt = segmentToPose(seg) ;

            boolean atend = false ;

            double dist = endpt.getTranslation().getDistance(pt.loc()) ;
            if (dist < 0.1)
                atend = true ;
            ret = new LookAheadPoint(endpt, atend) ;
        }

        return ret ;
    }
    
    private Pose2d segmentToPose(XeroPathSegment seg) {
        return new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
    }
    
    private Pose2d interpolate(double pcnt, Pose2d p1, Pose2d p2) {
        double x = p1.getX() + (p2.getX() - p1.getX()) * pcnt ;
        double y = p1.getY() + (p2.getY() - p1.getY()) * pcnt ;
        double heading = p1.getRotation().getRadians() + (p2.getRotation().getRadians() - p1.getRotation().getRadians()) * pcnt ;

        return new Pose2d(x, y, new Rotation2d(heading)) ;
    }
}
