package org.xero1425.base.tankdrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.MissingPathException;
import org.xero1425.misc.PIDACtrl;
import org.xero1425.misc.XeroMath;
import org.xero1425.misc.XeroPathSegment;

public class TankDrivePathFollowerAction extends TankDrivePathAction {    
    private int index_ ;
    private double left_start_ ;
    private double right_start_ ;
    private double start_time_ ;
    double angle_correction_clockwise ;
    double angle_correction_counter_clockwise ;
    PIDACtrl left_follower_ ;
    PIDACtrl right_follower_ ;
    boolean reverse_ ;
    double start_angle_ ;
    double target_start_angle_ ;
    private int plot_id_ ;
    Double [] plot_data_ ;
    static final String[] plot_columns_ = {             
        "time", 
        "ltpos", "lapos", "ltvel", "lavel", "ltaccel", "laaccel", "lout","lticks","lvout","laout","lpout","ldout","lerr",
        "rtpos", "rapos", "rtvel", "ravel", "rtaccel", "raaccel", "rout","rticks","rvout","raout","rpout","rdout","rerr",
        "thead", "ahead", "angcorr","angerr"
    } ;

    private final int LeftSide = 0 ;
    private final int RightSide = 1 ;

    public TankDrivePathFollowerAction(TankDriveSubsystem drive, String path, boolean reverse)
            throws MissingPathException, BadParameterTypeException, MissingParameterException {
        super(drive, path) ;

        reverse_ = reverse ;

        left_follower_ = new PIDACtrl(drive.getRobot().getSettingsParser(), "tankdrive:follower:left", false) ;
        right_follower_ = new PIDACtrl(drive.getRobot().getSettingsParser(), "tankdrive:follower:right", false) ;
        angle_correction_clockwise = drive.getRobot().getSettingsParser().get("tankdrive:follower:angle_correction:clockwise").getDouble() ;
        angle_correction_counter_clockwise = drive.getRobot().getSettingsParser().get("tankdrive:follower:angle_correction:counter_clockwise").getDouble() ;

        plot_id_ = drive.initPlot(toString(0)) ;
        plot_data_ = new Double[plot_columns_.length] ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        left_start_ = getSubsystem().getLeftDistance() ;
        right_start_ = getSubsystem().getRightDistance() ;

        index_ = 0 ;
        start_time_ = getSubsystem().getRobot().getTime() ;
        start_angle_ = getSubsystem().getAngle() ;
        target_start_angle_ = getPath().getSegment(LeftSide, 0).getHeading() ;

        getSubsystem().startPlot(plot_id_, plot_columns_);
        getSubsystem().startTrip("pathfollower") ;

        getSubsystem().startPlot(plot_id_, plot_columns_);
    }

    @Override
    public void run() {
        TankDriveSubsystem td = getSubsystem();
        XeroRobot robot = td.getRobot() ;

        MessageLogger logger = robot.getMessageLogger();
        logger.startMessage(MessageType.Debug) ;
        logger.add("index", index_) ;

        if (index_ < getPath().getSize())
        {
            double dt = robot.getDeltaTime();
            XeroPathSegment lseg = getPath().getSegment(LeftSide, index_) ;
            XeroPathSegment rseg = getPath().getSegment(RightSide, index_) ;

            double laccel, lvel, lpos ;
            double raccel, rvel, rpos ;
            double thead, ahead ;

            if (reverse_)
            {
                laccel = -rseg.getAccel() ;
                lvel = -rseg.getVelocity() ;
                lpos = -rseg.getPosition() ;
                raccel = -lseg.getAccel() ;
                rvel = -lseg.getVelocity() ;
                rpos = -lseg.getPosition() ;
                thead = XeroMath.normalizeAngleDegrees(lseg.getHeading() - target_start_angle_) ;
                ahead = XeroMath.normalizeAngleDegrees(getSubsystem().getAngle() - start_angle_) ;                 
            }
            else
            {
                laccel = lseg.getAccel() ;
                lvel = lseg.getVelocity() ;
                lpos = lseg.getPosition() ;
                raccel = rseg.getAccel() ;
                rvel = rseg.getVelocity() ;
                rpos = rseg.getPosition() ;
                thead = XeroMath.normalizeAngleDegrees(lseg.getHeading() - target_start_angle_) ;  
                ahead = XeroMath.normalizeAngleDegrees(getSubsystem().getAngle() - start_angle_) ;
            }

            double ldist, rdist ;
            ldist = td.getLeftDistance() - left_start_ ;
            rdist = td.getRightDistance() - right_start_ ;
            
            double lout = left_follower_.getOutput(laccel, lvel, lpos, ldist, dt) ;
            double rout = right_follower_.getOutput(raccel, rvel, rpos, rdist, dt) ;

            //
            // Negative angle is clockwise
            //
            double angerr = XeroMath.normalizeAngleDegrees(thead - ahead) ;

            double angcorr = 0.0 ;
            if (angerr > 0.0)
            {
                //
                // Angle is negative, clockwise to desired heading.  Expect the param from
                // the params file to be negative so we reduce the left side power and increase
                // the right side power
                //
                angcorr = angle_correction_counter_clockwise * angerr ;
            }
            else
            {
                //
                // Angle is positive, counter-clockwise to desired heading.  Expect the param from
                // the params file to be positive so we increase the left side power and reduce
                // the right side power
                //
                angcorr = angle_correction_clockwise * angerr ;
            }

            lout += angcorr ;
            rout -= angcorr ;

            td.setPower(lout, rout) ;
            logger.add(", left", lout) ;
            logger.add(", right", rout) ;
            logger.add(", angerr(degs)", angerr) ;
            logger.add(", angcorr(v)", angcorr) ;

            plot_data_[0] = robot.getTime() - start_time_ ;

            // Left side
            plot_data_[1] = lpos ;
            plot_data_[2] = td.getLeftDistance() - left_start_ ;
            plot_data_[3] = lvel ;
            plot_data_[4] = td.getLeftVelocity() ;
            plot_data_[5] = laccel ;
            plot_data_[6] = td.getLeftAcceleration() ;
            plot_data_[7] = lout ;
            plot_data_[8] = (double)td.getLeftTick() ;
            plot_data_[9] = left_follower_.getVPart() ;
            plot_data_[10] = left_follower_.getAPart() ;
            plot_data_[11] = left_follower_.getPPart() ;
            plot_data_[12] = left_follower_.getDPart() ;
            plot_data_[13] = left_follower_.getLastError() ;                                                

            // Right side
            plot_data_[14] = rpos ;
            plot_data_[15] = td.getRightDistance() - right_start_ ;
            plot_data_[16] = rvel ;
            plot_data_[17] = td.getRightVelocity() ;
            plot_data_[18] = raccel ;
            plot_data_[19] = td.getRightAcceleration() ;
            plot_data_[20] = rout ;
            plot_data_[21] = (double)td.getRightTick() ;
            plot_data_[22] = right_follower_.getVPart() ;
            plot_data_[23] = right_follower_.getAPart() ;
            plot_data_[24] = right_follower_.getPPart() ;
            plot_data_[25] = right_follower_.getDPart() ;
            plot_data_[26] = right_follower_.getLastError() ;

            plot_data_[27] = thead ;
            plot_data_[28] = ahead ;
            plot_data_[29] = angcorr ;
            plot_data_[30] = angerr ;

            td.addPlotData(plot_id_, plot_data_);
        }
        logger.endMessage();
        index_++ ;

        if (index_ == getPath().getSize())
        {
            td.endPlot(plot_id_);
            td.setPower(0.0, 0.0) ;
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        index_ = getPath().getSize() ;

        getSubsystem().setPower(0.0, 0.0) ;
        getSubsystem().endPlot(plot_id_);
    }

    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "TankDriveFollowPath-" + getPathName() ;
        return ret ;
    }
}