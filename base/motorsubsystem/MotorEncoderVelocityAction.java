package org.xero1425.base.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.PidType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;
import org.xero1425.misc.SettingsParser;

public class MotorEncoderVelocityAction extends MotorAction {   
    private static int which_ = 1 ;

    private double target_ ;
    private double start_ ;
    private PIDCtrl pid_ ;
    private int plot_id_ ;
    private static String [] columns_ = { "time", "target", "actual"}  ;

    public MotorEncoderVelocityAction(MotorEncoderSubsystem sub, double target)
            throws MissingParameterException, BadParameterTypeException, BadMotorRequestException {

        super(sub);

        target_ = target;

        if (!sub.hasHWPID()) {
            pid_ = new PIDCtrl(sub.getRobot().getSettingsParser(), sub.getName() + ":velocity", false);
            plot_id_ = sub.initPlot(toString() + "-" + String.valueOf(which_++)) ;     
        }
    }

    public MotorEncoderVelocityAction(MotorEncoderSubsystem sub, String target)
            throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        target_ = getSubsystem().getRobot().getSettingsParser().get(target).getDouble() ;

        if (!sub.hasHWPID()) {
            pid_ = new PIDCtrl(getSubsystem().getRobot().getSettingsParser(), sub.getName() + ":velocity", false);
            plot_id_ = - 1 ;
        }
    }

    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        target_ = target ;

        if (getSubsystem().getMotorController().hasPID()) {
            getSubsystem().getMotorController().setTarget(target);
        }
    }

    public double getTarget() {
        return target_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (getSubsystem().getMotorController().hasPID()) {
            SettingsParser settings = getSubsystem().getRobot().getSettingsParser() ;
            double p = settings.get("shooter:velocity:kp").getDouble() ;
            double i = settings.get("shooter:velocity:ki").getDouble() ;
            double d = settings.get("shooter:velocity:kd").getDouble() ;
            double f = settings.get("shooter:velocity:kf").getDouble() ;
            double outmax = settings.get("shooter:velocity:maxmagnitude").getDouble() ;

            getSubsystem().getMotorController().setPID(PidType.Velocity, p, i, d, f, outmax);
            getSubsystem().getMotorController().setTarget(target_) ;
        }
        else {
            pid_.reset() ;
            start_ = getSubsystem().getRobot().getTime() ;

            if (plot_id_ != -1)
                getSubsystem().startPlot(plot_id_, columns_) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
        if (!getSubsystem().getMotorController().hasPID()) {
            double out = pid_.getOutput(target_, me.getVelocity(), getSubsystem().getRobot().getDeltaTime()) ;
            getSubsystem().setPower(out) ;

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("MotorEncoderVelocityAction:") ;
            logger.add("target", target_) ;
            logger.add("actual", me.getVelocity()) ;
            logger.add("output", out) ;
            logger.add("encoder", me.getEncoderRawCount()) ;
            logger.endMessage();

            if (plot_id_ != -1) {
                Double[] data = new Double[columns_.length] ;
                data[0] = getSubsystem().getRobot().getTime() - start_ ;
                data[1] = target_ ;
                data[2] = me.getVelocity() ;
                getSubsystem().addPlotData(plot_id_, data);

                if (getSubsystem().getRobot().getTime() - start_ > 10.0)
                {
                    getSubsystem().endPlot(plot_id_) ;
                    plot_id_ = -1 ;
                }
            }
        }
        else
        {
            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("MotorEncoderVelocityAction:") ;
            logger.add("target", target_) ;
            logger.add("actual", me.getVelocity()) ;
            logger.add("inputV", me.getMotorController().getInputVoltage()) ;
            logger.add("appliedV", me.getMotorController().getAppliedVoltage()) ;
            logger.endMessage();            
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        try {
            if (getSubsystem().getMotorController().hasPID()) {
                getSubsystem().getMotorController().stopPID() ;
            }
        }
        catch(Exception ex) {
        }
        getSubsystem().setPower(0.0);
    }

    @Override
    public String toString(int indent) {
        String ret = null ;

        try {
            if (getSubsystem().getMotorController().hasPID()) {
                ret = prefix(indent) + "MotorEncoderVelocityAction(HWPID), " + getSubsystem().getName() + ", " +  Double.toString(target_) ;
            }
        }
        catch(Exception ex) {
        }

        if (ret == null) {
            ret = prefix(indent) + "MotorEncoderVelocityAction, " + getSubsystem().getName() + ", " +  Double.toString(target_) ;
        }

        return ret ;
    }
}