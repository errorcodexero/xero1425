package org.xero1425.base.motors ;

import java.util.List ;
import java.util.ArrayList ;

public class MotorGroupController extends MotorController
{ 
    List<MotorController> motors_ ;

    public MotorGroupController(String name) {
        super(name) ;
        motors_ = new ArrayList<MotorController>() ;
    }



    public String typeName() {
        if (motors_.size() == 0)
            return "NoMotors" ;
        
            return motors_.get(0).typeName() ;
    }

    public void addMotor(MotorController ctrl, boolean inverted) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() > 0 && !motors_.get(0).typeName().equals(ctrl.typeName()))
            throw new BadMotorRequestException(this, "cannot add motor to group with existing motors unless the are the same type") ;

        motors_.add(ctrl) ;

        if (motors_.size() > 1)
            ctrl.follow(motors_.get(0), inverted) ;
    }

    public double getInputVoltage() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
        throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getInputVoltage() ;
    }    
    public double getAppliedVoltage() throws BadMotorRequestException {
        if (motors_.size() == 0)
        throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getAppliedVoltage() ;
    }



    public boolean hasPID() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).hasPID() ;
    }

    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setTarget(target);
    }

    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setPID(type, p, i, d, f, outmax) ;
    }

    public void stopPID() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).stopPID() ;
    }

    public void setPositionConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setPositionConversion(factor);
    }

    public void setVelocityConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setVelocityConversion(factor);
    }

    public void set(double percent) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).set(percent) ;
    }

    public void setInverted(boolean inverted)  throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
            
        motors_.get(0).setInverted(inverted);
    }

    public boolean isInverted() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
            
        return motors_.get(0).isInverted() ;
    }

    public void reapplyInverted()  throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
            
        motors_.get(0).reapplyInverted();        
    }

    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        for(MotorController ctrl : motors_)
            ctrl.setNeutralMode(mode);
    }

    public void follow(MotorController ctrl, boolean invert) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "a motor group cannot follow other motors") ;
    }

    public String getType() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getType() ;
    }

    public boolean hasPosition() throws BadMotorRequestException{
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).hasPosition() ;
    }

    public double getPosition() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getPosition() ;  
    }

    public void resetEncoder() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).resetEncoder();
    }

    public void setCurrentLimit(double limit) throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
                    
        for(MotorController ctrl : motors_)
            ctrl.setCurrentLimit(limit);
    }      

    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
                    
        for(MotorController ctrl : motors_)
            ctrl.setOpenLoopRampRate(limit);
    }   

    public String getFirmwareVersion() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        StringBuilder result = new StringBuilder() ;

        for(int i = 0 ; i < motors_.size() ; i++) {
            if (result.length() > 0)
                result.append(",") ;
            
            result.append(motors_.get(i).getFirmwareVersion()) ;
        }

        return result.toString() ;
    }

    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        for(MotorController ctrl : motors_)
            ctrl.setEncoderUpdateFrequncy(freq);
    }
} ;
