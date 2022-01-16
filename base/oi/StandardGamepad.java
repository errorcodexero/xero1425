package org.xero1425.base.oi;

import org.xero1425.base.LoopType;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

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

    private double deadband_ ;

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
        deadband_ = getSubsystem().getSettingsValue(getName() + ":deadband").getDouble();
    }

    /// \brief generate the actions for the drivebase for the current robot loop
    @Override
    public void generateActions(SequenceAction seq) {

        if (db_ == null || isEnabled() == false)
          return ;

        double xSpeed = DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
        double zRotation = DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

        xSpeed = MathUtil.applyDeadband(xSpeed, deadband_) ;
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0) ;
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed) ;

        zRotation = MathUtil.applyDeadband(zRotation, deadband_) ;
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0) ;
        zRotation = Math.copySign(zRotation * zRotation, zRotation) ;        

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed) ;
        double leftSpeed, rightSpeed ;

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
              leftSpeed = maxInput;
              rightSpeed = xSpeed - zRotation;
            } else {
              leftSpeed = xSpeed + zRotation;
              rightSpeed = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
              leftSpeed = xSpeed + zRotation;
              rightSpeed = maxInput;
            } else {
              leftSpeed = maxInput;
              rightSpeed = xSpeed - zRotation;
            }
        }

        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }

        TankDrivePowerAction act = new TankDrivePowerAction(db_, leftSpeed, rightSpeed) ;
        try {
            seq.addSubActionPair(db_, act, false);
            left_ = leftSpeed ;
            right_ = rightSpeed ;
        } catch (InvalidActionRequest e) {
        }
    }
}