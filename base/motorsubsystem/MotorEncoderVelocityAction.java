package org.xero1425.base.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.PidType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;
import edu.wpi.first.wpilibj.RobotBase;

/// \file

/// \brief This action causes a MotorEncoderSubsystem to maintain a constant velocity.
public class MotorEncoderVelocityAction extends MotorAction {   

    // An index used to ensure individual instances of this action produce separate plots
    private static int which_ = 1 ;

    // The target velocity
    private double target_ ;

    // The start time for the action
    private double start_ ;

    // The PID controller to manage the velocity
    private PIDCtrl pid_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // The name of the action
    private String name_ ;

    // If true, force the PID controller to software, otherwise the PID loop will run in the
    // motor controller if the controller supports it
    private boolean forcesw_ ;

    // The columns to plot
    private static String [] columns_ = { "time", "target", "actual"}  ;

    /// \brief Create a new MotorEncoderVelocityAction
    /// \param sub the target MotorEncoderSubsystem
    /// \param name the name of the action, for entries from the settings file
    /// \param target the traget velocity
    public MotorEncoderVelocityAction(MotorEncoderSubsystem sub, String name, double target)
            throws MissingParameterException, BadParameterTypeException, BadMotorRequestException {

        super(sub);

        name_ = name ;
        target_ = target;
        forcesw_ = RobotBase.isSimulation();

        if (!sub.hasHWPID() || forcesw_) {
            pid_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), "subsystems:" + sub.getName() + ":" + name_, false);
            plot_id_ = sub.initPlot(toString() + "-" + String.valueOf(which_++)) ;     
        }
    }

    /// \brief Create a new MotorEncoderVelocityAction
    /// \param sub the target MotorEncoderSubsystem
    /// \param target a string with the name of the target velocity in settings file
    public MotorEncoderVelocityAction(MotorEncoderSubsystem sub, String target)
            throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        target_ = getSubsystem().getSettingsValue(target).getDouble() ;
        forcesw_ = RobotBase.isSimulation();

        if (!sub.hasHWPID() || forcesw_) {
            pid_ = new PIDCtrl(getSubsystem().getRobot().getSettingsSupplier(), "subsystems:" + sub.getName() + ":" + name_, false);
            plot_id_ = - 1 ;
        }
    }

    /// \brief Return the name of the action
    /// \returns the name of the action
    public String getName() {
        return name_ ;
    }

    /// \brief Update the target velocity to a new velocity
    /// \param target the target velocity desired
    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        target_ = target ;

        if (!useSWPID()) {
            //
            // If we are running the loop in the motor controller, commuincate the new target to the
            // motor controller
            //
            getSubsystem().getMotorController().setTarget(target);
        }
    }

    /// \brief Returns the current target
    /// \returns the current target
    public double getTarget() {
        return target_ ;
    }

    /// \brief Start the velocity action
    @Override
    public void start() throws Exception {
        super.start() ;

        if (!useSWPID()) {
            //
            // We are using a control loop in the motor controller, get the 
            ISettingsSupplier settings = getSubsystem().getRobot().getSettingsSupplier() ;
            double p = getSubsystem().getSettingsValue(name_ + ":kp").getDouble() ;
            double i = getSubsystem().getSettingsValue(name_ + ":ki").getDouble() ;
            double d = getSubsystem().getSettingsValue(name_ + ":kd").getDouble() ;
            double f = getSubsystem().getSettingsValue(name_ + ":kf").getDouble() ;
            double outmax = settings.get(name_ + ":max").getDouble() ;

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

    /// \brief Process the velocity action once per robot loop, adjusting the power as needed
    @Override
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
        if (useSWPID()) {
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

    /// \brief Cancel the velocity action, settings the power of the motor to zero
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
        if (plot_id_ != -1)
            getSubsystem().endPlot(plot_id_) ;
    }

    /// \brief return a human readable string for the action
    /// \param indent the amount of white space prior to the description
    /// \returns a human readable string for the action
    @Override
    public String toString(int indent) {
        String ret = null ;

        try {
            if (!useSWPID()) {
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

    private boolean useSWPID() throws BadMotorRequestException, MotorRequestFailedException {
        return forcesw_ || !getSubsystem().getMotorController().hasPID() ;
    }
}