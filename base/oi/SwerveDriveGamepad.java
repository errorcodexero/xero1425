package org.xero1425.base.oi;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;

public class SwerveDriveGamepad extends Gamepad {
    private SwerveDriveSubsystem db_ ;

    public SwerveDriveGamepad(OISubsystem oi, int index, DriveBaseSubsystem drive_) throws Exception {
        super(oi, "Xero1425GamePad", index);

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
    }

    @Override
    public void computeState() {
        super.computeState();
    }

    @Override
    public void generateActions(SequenceAction seq) {
        if (db_ == null || !isEnabled())
            return ;
    }    
}
