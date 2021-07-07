package org.xero1425.base.oi;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.swervedrive.SwerveDriveDirectionRotateAction;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;

public class SwerveDriveGamepad extends Gamepad {
    private SwerveDriveSubsystem db_;
    private double angle_nominal_;
    private double angle_maximum_;
    private double pos_nominal_;
    private double pos_maximum_;
    private double slowby_;
    private SwerveDriveDirectionRotateAction action_;

    public SwerveDriveGamepad(OISubsystem oi, int index, DriveBaseSubsystem drive_) throws Exception {
        super(oi, "Xero1425GamePad", index);

        slowby_ = 0.5 ;

        DriverStation ds = DriverStation.getInstance();
        if (ds.getStickPOVCount(getIndex()) == 0) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        if (ds.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        db_ = (SwerveDriveSubsystem)drive_;
    }
    
    @Override
    public void init(LoopType ltype) {
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        action_ = new SwerveDriveDirectionRotateAction(db_, 0.0, 0.0, 0.0) ;
    }

    @Override
    public void computeState() {
        super.computeState();
    }

    @Override
    public void generateActions(SequenceAction seq) {
        if (db_ == null || !isEnabled())
            return ;

        DriverStation ds = DriverStation.getInstance() ;
        double ly = ds.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
        double lx = ds.getStickAxis(getIndex(), AxisNumber.LEFTX.value) ;
        double rx = ds.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

        double lyscaled = scale(ly, 0.0, false, pos_nominal_, pos_maximum_, slowby_) ;
        double lxscaled = scale(lx, 0.0, false, pos_nominal_, pos_maximum_, slowby_) ;
        double rxscaled = scale(rx, 0.0, false, angle_nominal_, angle_maximum_, slowby_) ;

        //
        // The rotational velocity is given by rxscaled
        // The position velocity is given by the vector (lxscaled, lyscaled)
        //
        action_.updateTargets(lxscaled, lyscaled, rxscaled) ;
        if (db_.getAction() != action_)
            db_.setAction(action_) ;
    }    
}
