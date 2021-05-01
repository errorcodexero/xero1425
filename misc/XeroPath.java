package org.xero1425.misc ;

import java.util.ArrayList ;

/// \file

/// \brief This class represents a single path to be followed by the robot drive base
/// The XeroPath object has a name and a set of X and Y data points for both the left and right sides
/// of the drivebase
public class XeroPath
{
    //
    // The name of the path
    //
    private String name_ ;

    //
    // The type of the drivebase for the path
    //
    private XeroPathType dtype_ ;

    //
    // The set of segment for the left side of the robot
    //
    private ArrayList<ArrayList<XeroPathSegment>> data_ ;

    /// \brief create a new path with the name given
    /// \param name the name of the path
    public XeroPath(String name, XeroPathType dtype) throws Exception {
        name_ = name ;
        dtype_ = dtype ;
        data_ = new ArrayList<ArrayList<XeroPathSegment>>() ;

        if (dtype == XeroPathType.TankPathFollowing)
        {
            data_.add(new ArrayList<XeroPathSegment>()) ;
            data_.add(new ArrayList<XeroPathSegment>()) ;
        }
        else if (dtype == XeroPathType.SwervePathFollowing)
        {
            data_.add(new ArrayList<XeroPathSegment>()) ;
            data_.add(new ArrayList<XeroPathSegment>()) ;
            data_.add(new ArrayList<XeroPathSegment>()) ;
            data_.add(new ArrayList<XeroPathSegment>()) ;
        }
        else if (dtype == XeroPathType.TankPurePursuit)
        {
            data_.add(new ArrayList<XeroPathSegment>()) ;
        }
        else if (dtype == XeroPathType.SwervePurePursuit)
        {
            throw new Exception("not supported yet") ;
        }
    }

    /// \brief return the path type
    /// \returns the path type
    public XeroPathType getDriveType() {
        return dtype_ ;
    }

    /// \brief return the name of the path
    /// \returns the name of the path
    public String getName() {
        return name_ ;
    }

    /// \brief returns the number of data points in the path
    /// \returns the numer of data points in the path
    public int getSize() {
        return data_.get(0).size() ;
    }

    /// \brief returns the duration of the path in seconds
    /// \returns the duration of the path in seconds
    public double getDuration() {
        ArrayList<XeroPathSegment> seg = data_.get(0) ;
        return seg.get(seg.size() - 1).getTime() ;
    }

    /// \brief returns a single segment of the path for the requested side of the robot
    /// \param which which wheel to return data for
    /// \param index the index of the segment to return
    /// \returns a single segment of the path for the requested side of the robot
    public XeroPathSegment getSegment(int which, int index) {
        return data_.get(which).get(index) ;
    }

    /// \brief return the segment associated with each wheel
    /// \param index the index of the segment to return
    /// \returns the set of segments for all wheels for the given index
    public XeroPathSegment[] getSegments(int index)
    {
        XeroPathSegment[] ret = new XeroPathSegment[data_.size()] ;
        for(int i = 0 ; i < ret.length ; i++)
        {
            ret[i] = getSegment(i, index) ;
        }

        return ret ;
    }

    /// \brief adds a new path segment to the left adn right sides of the robot
    /// \param left the segment for the left side of the robot
    /// \param right the segment for the right side of the robot
    public void addPathSegment(int which, XeroPathSegment seg) throws Exception
    {
        if (which >= data_.size())
            throw new Exception("invalid wheel index in path") ;

        data_.get(which).add(seg) ;
    }

    /// \brief returns true if the path is valid
    /// \returns true if the path is valid
    public boolean isValid() {
        int size = data_.get(0).size() ;

        for(int i = 1 ; i < data_.size() ; i++)
        {
            if (data_.get(i).size() != size)
                return false ;
        }

        return true ;
    }
}
