package org.xero1425.base.swervedrive;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveDriveDirectionRotateAction extends SwerveDriveAction {
    private Translation2d dir_ ;
    private double rot_ ;
    private double circum_ ;
    private double [] angles_ ;
    private double [] speeds_ ;

    public SwerveDriveDirectionRotateAction(SwerveDriveSubsystem subsys, double x, double y, double rot) {
        super(subsys) ;

        dir_ = new Translation2d(x, y) ;
        rot_ = rot ;

        angles_ = new double[subsys.getModuleCount()] ;
        speeds_ = new double[subsys.getModuleCount()] ;

        for(int i = 0 ; i < subsys.getModuleCount() ; i++) {
            angles_[i] = 0.0 ;
            speeds_[i] = 0.0 ;
        }

        //
        // We approximate the circumference of the robot by averaging the length and width and applying
        // the CIRCUM = 2 * PI * R.  Calculating the circumference of an ellipse is more computational intensive
        // and really does not get us that much.
        //
        circum_ = (subsys.getLength() + subsys.getWidth()) * Math.PI ;
    }

    public void updateTargets(double dirx, double diry, double rot) {
        if (dirx != dir_.getX() || diry != dir_.getY() || rot != rot_)
        {
            dir_ = new Translation2d(dirx, diry) ;        
            rot_ = rot ;
        }
    }

    @Override
    public void start() throws Exception {
        super.start() ;
    }

    @Override
    public void run() {
        Translation2d dirrot = rotateVector(dir_, -getSubsystem().getAngle()) ;
        for(int i = 0 ; i < getSubsystem().getModuleCount() ; i++) {
            Translation2d rotvec = createRotVector(i, rot_);
            Translation2d resvec = addVectors(dirrot, rotvec) ;
            angles_[i] = Math.atan2(resvec.getY(), resvec.getX()) ;
            speeds_[i] = resvec.getNorm() ;
        }

        getSubsystem().setTargets(angles_, speeds_);
    }

    @Override
    public void cancel() {
        super.cancel() ;
        try {
            getSubsystem().stop() ;
        }
        catch(Exception ex) {

        }
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveDriveDirectionRotateAction:" ;
        ret += " dx " + Double.toString(dir_.getX()) ;
        ret += " dy " + Double.toString(dir_.getY()) ;
        ret += " rot " + Double.toString(rot_) ;
        return ret ;
    }

    private Translation2d rotateVector(Translation2d vec, double angle) {
        double rads = angle / 180.0 * Math.PI ;
        return new Translation2d(vec.getX() * Math.cos(rads) - vec.getY() * Math.sin(rads), vec.getY() * Math.cos(rads) + vec.getX() * Math.sin(rads)) ;
    }

    private Translation2d createRotVector(int which, double rot) {
        //
        // The rot value is in degress per second, we need to transform this to a
        // linear speed for the wheel
        //
        double linear = rot / 360.0 * circum_ ;
        double angle = 0.0 ;

        switch(which) {
            case SwerveDriveSubsystem.FL:
                angle = -45.0 ;
                break ;
            case SwerveDriveSubsystem.FR:
                angle = 45.0 ;
                break ;
            case SwerveDriveSubsystem.BL:
                angle = 45.0 ;
                break ;
            case SwerveDriveSubsystem.BR:
                angle = -45.0 ;
                break ;                                                
        }

        angle = angle / 180.0 * Math.PI ;
        return new Translation2d(Math.cos(angle) * linear, Math.sin(angle) * linear) ;
    }

    private Translation2d addVectors(Translation2d v1, Translation2d v2) {
        return new Translation2d(v1.getX() + v2.getX(), v1.getY() + v2.getY()) ;
    }
}
