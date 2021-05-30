package org.xero1425.base;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class PositionTracker {
    private Pose2d pose_ ;
    private double width_ ;
    private double scrub_ ;

    public PositionTracker(double width, double scrub) {
        width_ = width ;
        scrub_ = scrub ;
        pose_ = new Pose2d(0.0, 0.0, new Rotation2d(0.0)) ;
    }

    public Pose2d getPose() {
        return pose_ ;
    }

    public void setPose(Pose2d pose) {
        pose_ = pose ;
    }

    public double getWidth() {
        return width_ ;
    }

    public double getScrub() {
        return scrub_ ;
    }
        
    public void updatePosition(double dleft, double dright, double dangle) {

        double xpos, ypos ;



        double angle = dangle * Math.PI / 180.0 ;

        if (Math.abs(dleft - dright) < 1e-6) {
            xpos = pose_.getX() +  dleft * Math.cos(angle) ;
            ypos = pose_.getY() + dright * Math.sin(angle) ;
        }
        else {
            double r = width_ * (dleft + dright) / (2 * (dright - dleft)) ;
            double wd = (dright - dleft) / width_ ;
            xpos = pose_.getX() + r * Math.sin(wd + angle) - r * Math.sin(angle) ;
            ypos = pose_.getY() - r * Math.cos(wd + angle) + r * Math.cos(angle) ;
        }

        pose_ = new Pose2d(xpos, ypos, Rotation2d.fromDegrees(angle)) ;
    }
}
