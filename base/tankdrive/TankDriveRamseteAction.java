package org.xero1425.base.tankdrive;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class TankDriveRamseteAction extends TankDrivePathAction {
    private RamseteController ctrl_;
    private boolean reverse_ ;
    private int index_ ;

    private final int MainRobot = 0;
    private final double inchesToMeters = 0.0254 ;
    private final double kEpsilon = 1e-9 ;

    public TankDriveRamseteAction(TankDriveSubsystem sub, String pathname, boolean reverse, double b, double zeta) {
        super(sub, pathname);

        reverse_ = reverse ;
        ctrl_ = new RamseteController(b, zeta);
    }

    public TankDriveRamseteAction(TankDriveSubsystem sub, String pathname, boolean reverse) throws BadParameterTypeException, MissingParameterException {
        this(sub, pathname, reverse,
                sub.getRobot().getSettingsParser().get(sub.getName() + ":ramsete:b").getDouble(), 
                sub.getRobot().getSettingsParser().get(sub.getName() + ":ramsete:zeta").getDouble()) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        index_ = 0 ;

        XeroPathSegment seg = getPath().getSegment(MainRobot, 0) ;
        Pose2d start = new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
        getSubsystem().setPose(start);
    }

    @Override
    public void run() {

        if (index_ < getPath().getSize())
        {
            Pose2d currentPose = convertToMeters(getSubsystem().getPose()) ;
            XeroPathSegment seg = getPath().getSegment(MainRobot, index_) ;
            Pose2d desiredPose = convertToMeters(new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading()))) ;
            double linearVelocityRefMeters = convertToMeters(seg.getVelocity()) ;
            double angularVelocityRefRadiansPerSecond = 0.0 ;

            if (index_ != 0) {
                XeroPathSegment prev = getPath().getSegment(MainRobot, index_) ;
                angularVelocityRefRadiansPerSecond = (seg.getHeading() - prev.getHeading()) / getSubsystem().getRobot().getPeriod() / 180.0 * Math.PI ;
            }

            ChassisSpeeds speeds = ctrl_.calculate(currentPose, desiredPose, linearVelocityRefMeters, angularVelocityRefRadiansPerSecond) ;
            TankDriveVelocities vel = inverseKinematics(new Twist2d(speeds.vxMetersPerSecond, 0.0, speeds.omegaRadiansPerSecond)) ;
            getSubsystem().setPower(vel.getLeft(), vel.getRight()) ;
            index_++ ;
        }

        if (index_ == getPath().getSize())
        {
            getSubsystem().setPower(0.0, 0.0) ;
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        index_ = getPath().getSize() ;
        getSubsystem().setPower(0.0, 0.0) ;
    }
    
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "TankDriveRamseteAction-" + getPathName() ;
        return ret ;
    }

    private double convertToMeters(double v) {
        return v * inchesToMeters ;
    }

    private Pose2d convertToMeters(Pose2d input) {

        double x = input.getX() * inchesToMeters ;
        double y = input.getY() * inchesToMeters ;

        return new Pose2d(x, y, input.getRotation()) ;
    }

    private TankDriveVelocities inverseKinematics(Twist2d velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new TankDriveVelocities(velocity.dx, velocity.dx);
        }
        double delta_v = getSubsystem().getWidth() * velocity.dtheta / (2 * getSubsystem().getScrub());
        return new TankDriveVelocities(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}
