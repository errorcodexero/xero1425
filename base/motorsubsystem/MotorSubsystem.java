package org.xero1425.base.motorsubsystem;

import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class MotorSubsystem extends Subsystem
{
    private static final double epsilon = 1e-3 ;
    private MotorController controller_ ;
    private double power_ ;

    public MotorSubsystem(Subsystem parent, String name) {
        super(parent, name) ;

        String mname = "subsystems:" + name + ":hw:motors" ;
        controller_ = getRobot().getMotorFactory().createMotor(name, mname) ;
        if (controller_ == null)
        {
            getRobot().getMessageLogger().startMessage(MessageType.Fatal) ;
            getRobot().getMessageLogger().add("could not create motor for name '" + mname + "'") ;
            getRobot().getMessageLogger().endMessage();
        }
        power_ = 0.0 ;
    }

    public boolean isRunning() {
        return Math.abs(power_) > epsilon ;
    }

    @Override
    public void init(LoopType ltype) {
        super.init(ltype) ;
        try {
            controller_.reapplyInverted();
        }
        catch(BadMotorRequestException | MotorRequestFailedException ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("subsystem ").addQuoted(getName()).add(": cannot reapply inverted state -").add(ex.getMessage()).endMessage();
        }
    }

    @Override
    public void reset() {
        super.reset() ;
        setPower(0.0) ;
    }

    public SettingsValue getProperty(String name) {
        if (name.equals("power"))
            return new SettingsValue(getPower()) ;

        return null ;
    }

    public double getPower() {
        return power_ ;
    }

    public MotorController getMotorController() {
        return controller_ ;
    }

    protected void setPower(double p) {
        try {
            power_ = limitPower(p) ;
            controller_.set(power_) ;
        }
        catch(BadMotorRequestException|MotorRequestFailedException ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("subsystem ").addQuoted(getName()).add(": cannot set power -").add(ex.getMessage()).endMessage();
        }
    }

    protected double limitPower(double p) {
        return p ;
    }
} ;