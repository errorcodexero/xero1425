package org.xero1425.base.motors;

import edu.wpi.first.wpilibj.Spark;

public class RomiMotorController extends MotorController {

    private Spark motor_ ;
    private boolean inverted_ ;

    public RomiMotorController(String name, int index) {
        super(name) ;

        motor_ = new Spark(index) ;
    }

    public void set(double v) throws BadMotorRequestException {
        motor_.set(v) ;
    }

    public String typeName() {
        return "Romi" ;
    }

    public void setInverted(boolean inverted)  throws BadMotorRequestException {
        inverted_ = inverted ;
        motor_.setInverted(inverted);
    }

    public boolean isInverted() throws BadMotorRequestException {
        return inverted_ ;
    }



    public void reapplyInverted() throws BadMotorRequestException {
        motor_.setInverted(inverted_);
    }

    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException {

    }

    public String getType()  throws BadMotorRequestException {
        return "Romi" ;
    }

    public double getInputVoltage() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the getInputVoltage() capability") ;
    }

    public double getAppliedVoltage() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the getAppliedVoltage() capability") ;
    }

    public boolean hasPID() throws BadMotorRequestException {
        return false ;
    }

    public void setTarget(double target) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the PID capability") ;
    }

    public void stopPID() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the PID capability") ;
    }

    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the PID capability") ;        
    }

    public void follow(MotorController ctrl, boolean invert) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the follow() capability") ;
    }

    public void setPositionConversion(double conv) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the setPositionConversion() capability") ;
    }

    public void setVelocityConversion(double conv) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the setVelocityConversion() capability") ;
    }

    public String getFirmwareVersion() throws BadMotorRequestException {
        return "?.?" ;
    }
}
