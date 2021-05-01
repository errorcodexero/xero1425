package org.xero1425.base.utils;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class LineSegment {
    private Translation2d p1_ ;
    private Translation2d p2_ ;

    public LineSegment(Translation2d p1, Translation2d p2) {
        p1_ = new Translation2d(p1.getX(), p1.getY()) ;
        p2_ = new Translation2d(p2.getX(), p2.getY());
    }

    public LineSegment(double d, double e, double f, double g) {
        p1_ = new Translation2d(d, e) ;
        p2_ = new Translation2d(f, g) ;
    }

    public double length() {
        return p1_.getDistance(p2_) ;
    }

    public double dotProdParam(Translation2d pt)
    {
        Translation2d pt_p1 = pt.minus(p1_) ;
        Translation2d p2_p1 = p2_.minus(p1_) ;
        double dot = pt_p1.getX() * p2_p1.getX() + pt_p1.getY() * p2_p1.getY() ;
        double lensq = p2_p1.getX() * p2_p1.getX() + p2_p1.getY() * p2_p1.getY() ;
        double param = -1 ;

        if (lensq != 0.0)
            param = dot / lensq ;

        return param ;
    }

    public Translation2d closest(Translation2d pt) {
        Translation2d ret = null ;
        Translation2d pt_p1 = pt.minus(p1_) ;
        Translation2d p2_p1 = p2_.minus(p1_) ;
        double dot = pt_p1.getX() * p2_p1.getX() + pt_p1.getY() * p2_p1.getY() ;
        double lensq = p2_p1.getX() * p2_p1.getX() + p2_p1.getY() * p2_p1.getY() ;
        double param = -1 ;

        if (lensq != 0.0)
            param = dot / lensq ;

        if (param < 0.0) {
            //
            // The point is closeest to point P1
            //
            ret = p1_ ;
        }
        else if (param >= 0.0 && param <= 1.0)
        {
            double x = p1_.getX() + param * p2_p1.getX() ;
            double y = p1_.getY() + param * p2_p1.getY() ;

            ret = new Translation2d(x, y) ;
        }
        else
        {
            //
            // The point is closest to point P2
            //
            ret = p2_ ;
        }

        return ret ;
    }
}
