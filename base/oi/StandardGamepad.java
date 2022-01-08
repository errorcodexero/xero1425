package org.xero1425.base.oi;

import org.xero1425.base.LoopType;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This class controls interprets the input from the game pad to control the drivebase.
/// This class expects values stored in the JSON settings file.
///
///     {
///         "subsystems" : {
///             "oisubsystemname" : {
///                 "gamepadname" : {
///                 }
///             }
///         }
///     }
///
public class StandardGamepad extends Gamepad {
    
    // The drivebase subsystem to control
    private TankDriveSubsystem db_ ;

    // The current left drivebase power
    private double left_ ;

    // The current right drivebase power
    private double right_ ;

    /// \brief Create a new TankDrive gamepad device
    /// \param oi the subsystems that owns this device
    /// \param index the index to use when access this device in the WPILib library
    /// \param drive the drivebase to control
    public StandardGamepad(OISubsystem oi, int index, TankDriveSubsystem drive) throws Exception {
        super(oi, "standard_gamepad", index);

        db_ = drive;
    }

    /// \brief initialize the gamepad per robot mode, does nothing
    @Override
    public void init(LoopType ltype) {
    }

    public double getSum() {
        return left_ + right_ ;
    }

    public TankDriveSubsystem getDB() {
        return db_ ;
    }

    /// \brief create the required static actions
    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
    }

    /// \brief generate the actions for the drivebase for the current robot loop
    @Override
    public void generateActions(SequenceAction seq) {
    }

    // private double mapJoyStick(double v, double maxv, double db, double power) {
    //     if (Math.abs(v) < db)
    //         return 0.0 ;

    //     return Math.signum(v) * Math.pow(Math.abs(v), power) * maxv ;
    // }
}