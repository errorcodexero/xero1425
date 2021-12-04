package org.xero1425.base.oi;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.swervedrive.SwerveDirectionRotateAction;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsParser;

import edu.wpi.first.wpilibj.DriverStation;

public class SwerveDriveGamepad extends Gamepad {
    private SwerveDriveSubsystem db_;
    private double angle_maximum_;
    private double pos_maximum_;
    private double deadband_pos_x_ ;
    private double deadband_pos_y_ ;
    private double deadband_rotate_ ;
    private double power_ ;
    private SwerveDirectionRotateAction action_;

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
        ISettingsSupplier settings = getSubsystem().getRobot().getSettingsParser();

        deadband_pos_x_ = getSubsystem().getSettingsValue("driver:deadband:x").getDouble() ;
        deadband_pos_y_ = getSubsystem().getRobot().getSettingsParser().get("driver:deadband:y").getDouble() ;
        deadband_rotate_ = getSubsystem().getRobot().getSettingsParser().get("driver:deadband:rotate").getDouble() ;
        power_ = getSubsystem().getRobot().getSettingsParser().get("driver:axis:power").getDouble() ;
        
        angle_maximum_ = settings.get("driver:maximum:angle").getDouble();
        pos_maximum_ = settings.get("driver:maximum:position").getDouble();        

        action_ = new SwerveDirectionRotateAction(db_, 0.0, 0.0, 0.0) ;
    }

    @Override
    public void computeState() {
        super.computeState();
    }

    @Override
    public void generateActions(SequenceAction seq) {
        if (db_ == null || !isEnabled())
            return ;

        // For X axis, left is -1, right is +1
        // For Y axis, forward is -1, back is +1

        DriverStation ds = DriverStation.getInstance() ;
        double ly = ds.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
        double lx = ds.getStickAxis(getIndex(), AxisNumber.LEFTX.value) ;
        double rx = ds.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

        double lyscaled = mapJoyStick(ly, pos_maximum_, deadband_pos_y_, power_) ;
        double lxscaled = mapJoyStick(lx, pos_maximum_, deadband_pos_x_, power_) ;
        double rxscaled = mapJoyStick(rx, angle_maximum_, deadband_rotate_, power_) ;

        //
        // The rotational velocity is given by rxscaled
        // The position velocity is given by the vector (lyscaled, lxscaled)
        //
        // Note, the x and y are swapped because of the orientation of the gamepad versus the orientation of the
        // field.  The drive team is at the end of the field looking down the X axis.  So, when the Y axis on the
        // gamepad is pushed forward (negative value from the gamepad), the driver expects the robot to move along
        // the positive X axis of the field.  
        //
        action_.updateTargets(-lyscaled, -lxscaled, -rxscaled) ;
        if (db_.getAction() != action_)
            db_.setAction(action_) ;
    }    

    private double mapJoyStick(double v, double maxv, double db, double power) {
        if (Math.abs(v) < db)
            return 0.0 ;

        return Math.signum(v) * Math.pow(Math.abs(v), power) * maxv ;
    }
}
