package org.xero1425.base;

import org.xero1425.base.alarms.AlarmSubsystem;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.DriveBaseSubsystem;


public class RobotSubsystem extends Subsystem
{
    private OISubsystem oi_ ;
    private DriveBaseSubsystem db_ ;
    private AlarmSubsystem alarms_ ;

    public RobotSubsystem(XeroRobot robot, String name) throws Exception {
        super(robot, name) ;

        oi_ = null ;
        db_ = null ;
        alarms_ = new AlarmSubsystem(this) ;
        addChild(alarms_) ;
    }

    public void addChild(final Subsystem child) throws Exception {
        super.addChild(child) ;

        if (child.isOI()) {
            if (oi_ != null)
                throw new Exception("multiple OI subsystems added to robot subsystem") ;

            oi_ = (OISubsystem)child ;
        }
        else if (child.isDB()) {
            if (db_ != null)
                throw new Exception("multiple drivebase subsystems added to robot subsystem") ;

            db_ = (DriveBaseSubsystem)child ;
        }
    }

    public OISubsystem getOI() {
        return oi_ ;
    }

    public DriveBaseSubsystem getDB() {
        return db_ ;
    }

    public AlarmSubsystem getAlarms() {
        return alarms_ ;
    }

} ;