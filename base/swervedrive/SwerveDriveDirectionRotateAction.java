package org.xero1425.base.swervedrive;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveDriveDirectionRotateAction extends SwerveDriveAction {
    Translation2d dir_ ;
    double rot_ ;

    public SwerveDriveDirectionRotateAction(SwerveDriveSubsystem subsys, double dirx, double diry, double rot) {
        super(subsys) ;

        dir_ = new Translation2d(dirx, diry) ;
        rot_ = rot ;
    }

    public void updateTargets(double dirx, double diry, double rot) {
        dir_ = new Translation2d(dirx, diry) ;        
        rot_ = rot ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
    }

    @Override
    public void run() {
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
        ret += " dx" + Double.toString(dir_.getX()) ;
        ret += " dy" + Double.toString(dir_.getY()) ;
        ret += " rot" + Double.toString(rot_) ;
        return ret ;
    }
}
