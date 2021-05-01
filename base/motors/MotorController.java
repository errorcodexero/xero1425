package org.xero1425.base.motors ;

public abstract class MotorController
{
    public final static String SimPowerParamName = "Power" ;
    public final static String SimEncoderParamName = "Encoder" ;
    public final static String SimEncoderStoresTicksParamName = "StoresTicks" ;
    public final static String SimInvertedParamName = "Inverted" ;
    public final static String SimNeutralParamName = "Neutral" ;

    public enum NeutralMode { 
        Coast,
        Brake
    } ;

    MotorController(String name) {
        name_ = name ;
    }

    public String getName() {
        return name_  ;
    }

    public enum PidType {
        Position,
        Velocity,
    }

    public abstract String typeName() ;
    public abstract void set(double percent)  throws BadMotorRequestException, MotorRequestFailedException ;
    public abstract void setInverted(boolean inverted)  throws BadMotorRequestException, MotorRequestFailedException ;
    public abstract boolean isInverted() throws BadMotorRequestException , MotorRequestFailedException ;    
    public abstract void reapplyInverted() throws BadMotorRequestException, MotorRequestFailedException ;
    public abstract void setNeutralMode(NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException ;
    public abstract void follow(MotorController ctrl, boolean invert) throws BadMotorRequestException, MotorRequestFailedException ;
    public abstract String getType()  throws BadMotorRequestException, MotorRequestFailedException ;
    public abstract double getInputVoltage() throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract boolean hasPID() throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract void setTarget(double target) throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract void stopPID() throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract void setPositionConversion(double factor) throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract void setVelocityConversion(double factor) throws BadMotorRequestException , MotorRequestFailedException ;
    public abstract String getFirmwareVersion() throws BadMotorRequestException ;
    public abstract double getAppliedVoltage() throws BadMotorRequestException ;
    
    public boolean hasPosition() throws BadMotorRequestException {
        return false ;
    }

    public double getPosition() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support getPosition()") ;
    }

    public void resetEncoder() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support resetEncoder()") ;        
    }

    public void setCurrentLimit(double limit) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support setCurrentLimit()") ;        
    }

    public void setOpenLoopRampRate(double ramptime) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support setOpenLoopRampRate()") ;    
    }

    private String name_ ;
}
