package org.xero1425.base.tankdrive;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;
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
    private PIDCtrl left_pid_ ;
    private PIDCtrl right_pid_ ;

    private final int MainRobot = 0;
    private final double inchesToMeters = 0.0254 ;

    public TankDriveRamseteAction(TankDriveSubsystem sub, String pathname, boolean reverse, double b, double zeta)
            throws MissingParameterException, BadParameterTypeException {
        super(sub, pathname);

        reverse_ = reverse ;
        ctrl_ = new RamseteController(b, zeta);
        
        left_pid_ = new PIDCtrl(sub.getRobot().getSettingsParser(), sub.getName() + ":left", false) ;
        right_pid_ = new PIDCtrl(sub.getRobot().getSettingsParser(), sub.getName() + ":right", false) ;
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
            // Current pose in inches
            Pose2d currentPose = inchesToMeters(getSubsystem().getPose()) ;

            // Current segment in inches
            XeroPathSegment seg = getPath().getSegment(MainRobot, index_) ;

            // Desired pose in meters
            Pose2d desiredPose = inchesToMeters(new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading()))) ;

            // The desired linear velocity in meters
            double linearVelocityRefMeters = inchesToMeters(seg.getVelocity()) ;

            // The desired angular velocity in radians per second
            double angularVelocityRefRadiansPerSecond = 0.0 ;

            if (index_ != 0) {
                XeroPathSegment prev = getPath().getSegment(MainRobot, index_) ;

                // Compute the angular velocity in radians per second
                angularVelocityRefRadiansPerSecond = (seg.getHeading() - prev.getHeading()) / getSubsystem().getRobot().getPeriod() / 180.0 * Math.PI ;
            }

            // Robot speed in meters per second
            ChassisSpeeds speeds = ctrl_.calculate(currentPose, desiredPose, linearVelocityRefMeters, angularVelocityRefRadiansPerSecond) ;

            // Robot speed as a Twist2d in inches per second
            Twist2d twist = new Twist2d(metersToInches(speeds.vxMetersPerSecond), 0.0, speeds.omegaRadiansPerSecond) ;

            // Left and right speed in inches per second
            TankDriveVelocities vel = getSubsystem().inverseKinematics(twist) ;

            setVelocity(vel.getLeft(), vel.getRight()) ;
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

    //
    // Set the left and right wheel velocities in inches per second
    //
    private void setVelocity(double leftvel, double rightvel) {
        double left = left_pid_.getOutput(leftvel, getSubsystem().getLeftVelocity(), getSubsystem().getRobot().getDeltaTime()) ;
        double right = right_pid_.getOutput(rightvel, getSubsystem().getLeftVelocity(), getSubsystem().getRobot().getDeltaTime()) ;
        getSubsystem().setPower(left, right) ;
    }

    private double inchesToMeters(double v) {
        return v * inchesToMeters ;
    }

    private Pose2d inchesToMeters(Pose2d input) {

        double x = input.getX() * inchesToMeters ;
        double y = input.getY() * inchesToMeters ;

        return new Pose2d(x, y, input.getRotation()) ;
    }

    private double metersToInches(double v) {
        return v / inchesToMeters ;
    }
}
