package org.xero1425.base.oi;

import org.xero1425.base.LoopType;
import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsParser;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

public class NewDriveGamepad extends Gamepad {
    private final static double kEpsilon = 1e-9 ;
    private TankDriveSubsystem db_ ;
    private double left_ ;
    private double right_ ;
    private int pov_ ;
    private Action nudge_forward_ ;
    private Action nudge_backward_ ;
    private Action nudge_clockwise_ ;
    private Action nudge_counter_clockwise_ ;

    private int throttle_stick_ ;
    private int wheel_stick_ ;
    private double throttle_deadband_ ;
    private double wheel_deadband_ ;
    private double wheel_non_linearity_ ;
    private double demoninator_ ;
    private double wheel_gain_ ;
    private double delta_power_ ;

    public NewDriveGamepad(OISubsystem oi, int index, TankDriveSubsystem drive_) throws Exception {
        super(oi, index);

        SettingsParser settings = drive_.getRobot().getSettingsParser() ;
        DriverStation ds = DriverStation.getInstance();
        if (ds.getStickPOVCount(getIndex()) == 0) {
            MessageLogger logger = oi.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("driver gamepad does not have POV control - nudges are disabled");
            logger.endMessage();
            pov_ = -1;
        } else {
            pov_ = 0;
        }

        if (ds.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            MessageLogger logger = oi.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("driver gamepad does not have required number of axis, must have six");
            logger.endMessage();
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        throttle_stick_ = settings.get("newdrive:throttle:stick").getInteger() ;
        throttle_deadband_ = settings.get("newdrive:throttle:deadband").getDouble() ;
        wheel_stick_  = settings.get("newdrive:wheel:stick").getInteger() ;
        wheel_deadband_ = settings.get("newdrive:wheel:deadband").getDouble() ;
        wheel_non_linearity_ = settings.get("newdrive:wheel:nonlinearity").getDouble() ;
        wheel_gain_ = settings.get("newdrive:wheel:wheelgain").getDouble() ;
        delta_power_ = settings.get("newdrive:wheel:deltapower").getDouble() ;
        demoninator_ = Math.sin(Math.PI / 2.0 * wheel_non_linearity_) ;

        db_ = drive_;
    }
    @Override
    public void init(LoopType ltype) {
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        SettingsParser settings = getSubsystem().getRobot().getSettingsParser();

        double nudge_straight = settings.get("newdrive:power:nudge_straight").getDouble();
        double nudge_rotate = settings.get("newdrive:power:nudge_rotate").getDouble();
        double nudge_time = settings.get("newdrive:nudge_time").getDouble();

        nudge_forward_ = new TankDrivePowerAction(db_, nudge_straight, nudge_straight, nudge_time);
        nudge_backward_ = new TankDrivePowerAction(db_, -nudge_straight, -nudge_straight, nudge_time);
        nudge_clockwise_ = new TankDrivePowerAction(db_, -nudge_rotate, nudge_rotate, nudge_time);
        nudge_counter_clockwise_ = new TankDrivePowerAction(db_, nudge_rotate, -nudge_rotate, nudge_time);
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

            if (povvalue == POVAngle.LEFT)
                seq.addSubActionPair(db_, nudge_clockwise_, false);
            else if (povvalue == POVAngle.RIGHT)
                seq.addSubActionPair(db_, nudge_counter_clockwise_, false);       
            else if (povvalue == POVAngle.UP)
                seq.addSubActionPair(db_, nudge_forward_, false);  
            else if (povvalue == POVAngle.DOWN)
                seq.addSubActionPair(db_, nudge_backward_, false);                                       
            else
                drive(seq) ;
        }
        catch(InvalidActionRequest ex) {
            //
            // This should never happen
            //
        }
    }

    private void drive(SequenceAction seq) throws InvalidActionRequest {
        DriverStation ds = DriverStation.getInstance() ;
        double throttle = ds.getStickAxis(getIndex(), throttle_stick_) ;
        double wheel = ds.getStickAxis(getIndex(), wheel_stick_) ;
        boolean quickturn = isRBackButtonPressed() ;

        if (Math.abs(wheel) < wheel_deadband_)
            wheel = 0.0 ;

        if (Math.abs(throttle) < throttle_deadband_)
            throttle = 0.0 ;

        if (!quickturn)
        {
            wheel = Math.sin(Math.PI / 2.0 * wheel_non_linearity_ * wheel) ;
            wheel = Math.sin(Math.PI / 2.0 * wheel_non_linearity_ * wheel) ;
            wheel = wheel / (demoninator_ * demoninator_) * Math.abs(throttle) ;
        }
        else
        {
            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.add("QUick Turn") ;
            logger.endMessage();
        }

        wheel *= wheel_gain_ ;
        Translation2d drive = inverseKinematics(new Twist2d(throttle, 0.0, wheel)) ;
        double maxleftright = Math.max(Math.abs(drive.getX()), Math.abs(drive.getY())) ;
        double scale = Math.max(1.0, maxleftright) ;

        double left = drive.getX() / scale ;
        double right = drive.getY() / scale ;

        if (Math.abs(left - left_) > delta_power_ || Math.abs(right - right_) > delta_power_)
        {
            TankDrivePowerAction act = new TankDrivePowerAction(db_, left, right) ;
            seq.addSubActionPair(db_, act, false);
            left_ = left ;
            right_ = right ;
        }
    }

    Translation2d inverseKinematics(Twist2d velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new Translation2d(velocity.dx, velocity.dx);
        }
        double delta_v = db_.getWidth() * velocity.dtheta / (2 * db_.getScrub());
        return new Translation2d(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}
