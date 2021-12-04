package org.xero1425.base.oi;

import edu.wpi.first.wpilibj.DriverStation;
import org.xero1425.base.LoopType;
import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

/// \brief This class controls interprets the input from the game pad to control the drivebase.
///
/// This class has requirements for the settings file.  The following entries must be in the settings file or
/// this class will not work properly
///
///
public class TankDriveGamepad extends Gamepad {
    
    private TankDriveSubsystem db_ ;
    private Action nudge_forward_ ;
    private Action nudge_backward_ ;
    private Action nudge_clockwise_ ;
    private Action nudge_counter_clockwise_ ;
    private int pov_ ;

    private double default_power_ ;
    private double max_power_ ;
    private double turn_power_ ;
    private double turn_max_power_ ;
    private double slow_factor_ ;
    private double zero_level_ ;
    private double tolerance_ ;

    double left_ ;
    double right_ ;

    public TankDriveGamepad(OISubsystem oi, int index, TankDriveSubsystem drive_) throws Exception {
        super(oi, "Xero1425GamePad", index);

        DriverStation ds = DriverStation.getInstance();
        if (ds.getStickPOVCount(getIndex()) == 0) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        if (ds.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        db_ = drive_;
    }

    @Override
    public void init(LoopType ltype) {
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {

        default_power_ = getSubsystem().getSettingsValue("gamepad:power:default").getDouble();
        max_power_ = getSubsystem().getSettingsValue("gamepad:power:max").getDouble();
        turn_power_ = getSubsystem().getSettingsValue("gamepad:turn:default").getDouble();
        turn_max_power_ = getSubsystem().getSettingsValue("gamepad:turn:max").getDouble();
        slow_factor_ = getSubsystem().getSettingsValue("gamepad:power:slowby").getDouble();
        zero_level_ = getSubsystem().getSettingsValue("gamepad:zerolevel").getDouble();

        tolerance_ = getSubsystem().getSettingsValue("gamepad:power:tolerance").getDouble();

        double nudge_straight = getSubsystem().getSettingsValue("gamepad:power:nudge_straight").getDouble();
        double nudge_rotate = getSubsystem().getSettingsValue("gamepad:power:nudge_rotate").getDouble();
        double nudge_time = getSubsystem().getSettingsValue("gamepad:nudge_time").getDouble();

        nudge_forward_ = new TankDrivePowerAction(db_, nudge_straight, nudge_straight, nudge_time);
        nudge_backward_ = new TankDrivePowerAction(db_, -nudge_straight, -nudge_straight, nudge_time);
        nudge_clockwise_ = new TankDrivePowerAction(db_, nudge_rotate, -nudge_rotate, nudge_time);
        nudge_counter_clockwise_ = new TankDrivePowerAction(db_, -nudge_rotate, nudge_rotate, nudge_time);
    }

    @Override
    public void computeState() {
        super.computeState();
    }

    @Override
    public void generateActions(SequenceAction seq) {
        if (db_ == null || !isEnabled())
            return ;

        try {

            POVAngle povvalue ;

            DriverStation ds = DriverStation.getInstance() ;
            if (pov_ == -1)
                povvalue = POVAngle.NONE ;
            else
                povvalue = POVAngle.fromInt(ds.getStickPOV(getIndex(), pov_)) ;

            double ly = ds.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
            double rx = ds.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

            if (povvalue == POVAngle.LEFT)
                seq.addSubActionPair(db_, nudge_counter_clockwise_, false);
            else if (povvalue == POVAngle.RIGHT)
                seq.addSubActionPair(db_, nudge_clockwise_, false);       
            else if (povvalue == POVAngle.UP)
                seq.addSubActionPair(db_, nudge_forward_, false);  
            else if (povvalue == POVAngle.DOWN)
                seq.addSubActionPair(db_, nudge_backward_, false);                                       
            else {
                double left, right ;

                if (Math.abs(ly) < zero_level_ && Math.abs(rx) < zero_level_) {
                    left = 0.0 ;
                    right = 0.0; 
                }
                else {
                    double boost = ds.getStickAxis(getIndex(), AxisNumber.LTRIGGER.value) ;
                    boolean slow = isLBackButtonPressed() ;

                    double power = scalePower(-ly, boost, slow) ;
                    double spin = (Math.abs(rx) > 0.01) ? scaleTurn(rx, boost, slow) : 0.0 ;

                    left = power + spin ;
                    right = power - spin ;                    
                }

                if (Math.abs(left - left_) > tolerance_ || Math.abs(right - right_) > tolerance_)
                {
                    TankDrivePowerAction act = new TankDrivePowerAction(db_, left, right) ;
                    seq.addSubActionPair(db_, act, false);
                    left_ = left ;
                    right_ = right ;
                }
            }
        }
        catch(InvalidActionRequest ex) {
            //
            // This should never happen
            //
        }
    }

    private double scalePower(double axis, double boost, boolean slow) {
        double base = default_power_ + (max_power_ - default_power_) * boost ;
        double slowdown = slow ? default_power_ * slow_factor_ : 0.0 ;
        return axis * (base - slowdown) ;
    }

    private double scaleTurn(double axis, double boost, boolean slow) {
        double base = turn_power_ + (turn_max_power_ - turn_power_) * boost ;
        double slowdown = slow ? turn_power_ * slow_factor_ : 0.0 ;
        return axis * (base - slowdown) ;
    }
}