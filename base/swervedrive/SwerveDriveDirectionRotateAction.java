package org.xero1425.base.swervedrive;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveDriveDirectionRotateAction extends SwerveDriveAction {
    private Translation2d dir_ ;
    private double rot_ ;
    private double circum_ ;

    public SwerveDriveDirectionRotateAction(SwerveDriveSubsystem subsys, double x, double y, double rot) {
        super(subsys) ;

        dir_ = new Translation2d(x, y) ;
        rot_ = rot ;

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
        for(int i = 0 ; i < getSubsystem().getModuleCount() ; i++) {

        }
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
