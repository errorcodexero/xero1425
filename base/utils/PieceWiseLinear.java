package org.xero1425.base.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.SettingsParser;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class PieceWiseLinear {
    private List<Translation2d> points_;

    public PieceWiseLinear(SettingsParser settings, String basename) throws Exception {
        List<Translation2d> points = readFromSettings(settings, basename);
        init(points) ;
    }

    public PieceWiseLinear(List<Translation2d> points) throws Exception {
        init(points) ;
    }

    public int size() {
        return points_.size();
    }

    public Translation2d get(int i) {
        return points_.get(i);
    }

    public double getValue(double x) {
        double ret;

        if (x < points_.get(0).getX()) {
            ret = points_.get(0).getY();
        } else if (x > points_.get(points_.size() - 1).getX()) {
            ret = points_.get(points_.size() - 1).getY();
        } else {
            int which_x = findWhich(x);

            Translation2d low = points_.get(which_x);
            Translation2d high = points_.get(which_x + 1);

            double m = ((high.getY() - low.getY()) / (high.getX() - low.getX()));
            double b = high.getY() - (high.getX() * m);

            ret = (m * x) + b;

        }

        return ret;
    }

    private void init(List<Translation2d> points) throws Exception {
        points_ = new ArrayList<Translation2d>();

        if (points.size() == 0)
            throw new Exception("cannot have empty points list as input");

        for (int i = 0; i < points.size(); i++)
            points_.add(points.get(i));

        Collections.sort(points_, new Comparator<Translation2d>() {
            public int compare(Translation2d p1, Translation2d p2) {
                if (p1.getX() < p2.getX())
                    return -1;

                if (p1.getX() > p2.getX())
                    return 1;

                if (p1.getY() < p2.getY())
                    return -1;

                if (p1.getY() > p2.getY())
                    return 1;

                return 0;
            }
        });
    }

    private int findWhich(double x) {
        for (int i = 0; i < points_.size(); i++) {
            if (x >= points_.get(i).getX() && x < points_.get(i + 1).getX()) {
                return i;
            }
        }

        return -1;
    }

    private List<Translation2d> readFromSettings(SettingsParser settings, String base)
            throws BadParameterTypeException {
        int index = 1 ;
        List<Translation2d> points = new ArrayList<Translation2d>() ;

        while (true) {
            String xname = base + ":" + Integer.toString(index) + ":x" ;
            String yname = base + ":" + Integer.toString(index) + ":y" ;

            SettingsValue xvalue = settings.getOrNull(xname) ;
            SettingsValue yvalue = settings.getOrNull(yname) ;

            if (xvalue == null || yvalue == null)
                break ;

            Translation2d pt = new Translation2d(xvalue.getDouble(), yvalue.getDouble()) ;
            points.add(pt) ;

            index++ ;
        }

        return points ;
    }
}