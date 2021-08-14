package org.xero1425.base.swervedrive;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.Subsystem.DisplayType;
import org.xero1425.misc.XeroMath;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class SwerveNewPathAction extends SwerveDriveAction {
    private Trajectory traj_ ;
    private RamseteController ctrl_ ;
    private double start_ ;

    public SwerveNewPathAction(SwerveDriveSubsystem sub) {
        super(sub) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        List<Pose2d> pts  = new ArrayList<Pose2d>() ;

        pts.add(new Pose2d(XeroMath.inchesToMeters(0.0), XeroMath.inchesToMeters(180.0), Rotation2d.fromDegrees(0.0))) ;
        pts.add(new Pose2d(XeroMath.inchesToMeters(120.0), XeroMath.inchesToMeters(60.0), Rotation2d.fromDegrees(-90.0))) ;

        TrajectoryConfig cfg = new TrajectoryConfig(XeroMath.inchesToMeters(140), XeroMath.inchesToMeters(140)) ;
        traj_ = TrajectoryGenerator.generateTrajectory(pts, cfg) ;
        start_ = getSubsystem().getRobot().getTime() ;
        ctrl_ = new RamseteController() ;

        getSubsystem().startOdometry(DriveBaseSubsystem.metersToInches(traj_.getInitialPose())) ;
    }

    @Override
    public void run() {
        double elapsed = getSubsystem().getRobot().getTime() - start_ ;
        if (elapsed < traj_.getTotalTimeSeconds())
        {
            Pose2d currentPoseInches = getSubsystem().getPose() ;
            getSubsystem().putDashboard("db-trk-t", DisplayType.Always, getSubsystem().getRobot().getTime()) ;
            getSubsystem().putDashboard("db-trk-x", DisplayType.Always, currentPoseInches.getX()) ;
            getSubsystem().putDashboard("db-trk-y", DisplayType.Always, currentPoseInches.getY()) ;
            getSubsystem().putDashboard("db-trk-a", DisplayType.Always, currentPoseInches.getRotation().getDegrees()) ;
            
            Trajectory.State st = traj_.sample(elapsed) ;
            ChassisSpeeds speed = ctrl_.calculate(getSubsystem().getPoseMeters(), st) ;
            getSubsystem().setChassisSpeeds(speed) ;

            Pose2d pathpose = DriveBaseSubsystem.metersToInches(st.poseMeters) ;
            getSubsystem().putDashboard("db-path-t", DisplayType.Always, getSubsystem().getRobot().getTime()) ;
            getSubsystem().putDashboard("db-path-x", DisplayType.Always, pathpose.getX()) ;
            getSubsystem().putDashboard("db-path-y", DisplayType.Always, pathpose.getY()) ;
            getSubsystem().putDashboard("db-path-a", DisplayType.Always, pathpose.getRotation().getDegrees()) ;
        }
        else
        {
            setDone() ;
        }
    }

    @Override
    public void cancel() {
    }


    @Override
    public String toString(int ident) {
        return "SwerveNewPathAction" ;
    }
}
