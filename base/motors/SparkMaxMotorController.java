package org.xero1425.base.motors;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

public class SparkMaxMotorController extends MotorController
{
    private CANSparkMax controller_ ;
    private CANEncoder encoder_ ;
    private boolean inverted_ ;
    private boolean brushless_ ;
    private CANPIDController pid_ ;
    private PidType ptype_ ;
    private double target_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimDouble sim_encoder_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    public final static String SimDeviceNameBrushed = "SparkMaxBrushed" ;
    public final static String SimDeviceNameBrushless = "SparkMaxBrushless" ;
    public final static int TicksPerRevolution = 42 ;

    public SparkMaxMotorController(String name, int index, boolean brushless) throws MotorRequestFailedException {
        super(name) ;

        inverted_ = false ;
        brushless_ = brushless ;
        pid_ = null ;
        target_ = 0 ;

        if (RobotBase.isSimulation()) {
            if (brushless)
                sim_ = SimDevice.create(SimDeviceNameBrushless, index) ;
            else
                sim_ = SimDevice.create(SimDeviceNameBrushed, index) ;

            sim_power_ = sim_.createDouble(MotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_encoder_ = sim_.createDouble(MotorController.SimEncoderParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(MotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(MotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;  
            sim_.createBoolean(MotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, false) ;            
        }
        else {
            CANError code ;

            if (brushless)
            {
                controller_ = new CANSparkMax(index, CANSparkMax.MotorType.kBrushless) ;
            }
            else
            {
                controller_ = new CANSparkMax(index, CANSparkMax.MotorType.kBrushed) ;
            }

            code = controller_.restoreFactoryDefaults() ;
            if (code != CANError.kOk)
                throw new MotorRequestFailedException(this, "restoreFactoryDefaults() failed during initialization", code) ;

            code = controller_.enableVoltageCompensation(12.0) ;
            if (code != CANError.kOk)
                throw new MotorRequestFailedException(this, "enableVoltageCompensation() failed during initialization", code) ;

            encoder_ = controller_.getEncoder() ;
        }
    }



    public String typeName() {
        String ret = "SparkMaxBrushed" ;

        if (brushless_)
            ret = "SparkMaxBrushless" ;

        return ret ;
    }

    public double getInputVoltage() throws BadMotorRequestException {
        return controller_.getBusVoltage() ;
    }

    public double getAppliedVoltage() throws BadMotorRequestException {
        return controller_.getAppliedOutput() ;
    }

    public boolean hasPID() throws BadMotorRequestException {
        return true ;
    }

    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        CANError code = CANError.kOk ;

        target_ = target ;

        if (pid_ != null) {

            if (ptype_ == PidType.Position)
                code = pid_.setReference(target, ControlType.kPosition) ;
            else if (ptype_ == PidType.Velocity)
                code = pid_.setReference(target, ControlType.kVelocity) ;
            
            if (code != CANError.kOk)
                throw new MotorRequestFailedException(this, "setReference() failed during setTarget() call", code) ;
        }
    }

    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException,
            MotorRequestFailedException {
        CANError code = CANError.kOk ;

        if (pid_ == null)
            pid_ = controller_.getPIDController() ;

        code = pid_.setP(p) ;
        if (code != CANError.kOk)
            throw new MotorRequestFailedException(this, "setP() failed during setPID() call", code) ;

        code = pid_.setI(i) ;
        if (code != CANError.kOk)
            throw new MotorRequestFailedException(this, "setI() failed during setPID() call", code) ;        

        code = pid_.setD(d) ;
        if (code != CANError.kOk)
            throw new MotorRequestFailedException(this, "setD() failed during setPID() call", code) ;

        code = pid_.setFF(f) ;
        if (code != CANError.kOk)
            throw new MotorRequestFailedException(this, "setFF() failed during setPID() call", code) ;

        code = pid_.setIZone(0.0) ;
        if (code != CANError.kOk)
            throw new MotorRequestFailedException(this, "setIZone() failed during setPID() call", code) ;

        code = pid_.setOutputRange(-outmax, outmax) ;
        if (code != CANError.kOk)
            throw new MotorRequestFailedException(this, "setOutputRange() failed during setPID() call", code) ;

        ptype_ = type ;
        setTarget(target_) ;
    }

    public void stopPID() throws BadMotorRequestException {
    }

    public void setPositionConversion(double factor) throws BadMotorRequestException {
        encoder_.setPositionConversionFactor(factor) ;
    }

    public void setVelocityConversion(double factor) throws BadMotorRequestException {
        encoder_.setVelocityConversionFactor(factor) ;
    }

    public void set(double percent) {
        if (sim_ != null) {
            sim_power_.set(percent) ;
        } else {
            controller_.set(percent) ;
        }
    }

    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(inverted) ;
        } else {
            controller_.setInverted(inverted);
        }

        inverted_ = inverted ;
    }

    public boolean isInverted() {
        return inverted_ ;
    }    

    public void reapplyInverted() {
        if (sim_ != null) {
            sim_motor_inverted_.set(inverted_) ;
        } else {
            controller_.setInverted(inverted_);
        }
    }

    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException {
        if (sim_ != null) {
            switch(mode)
            {
                case Coast:
                    sim_neutral_mode_.set(false) ;
                    break ;

                case Brake:
                    sim_neutral_mode_.set(true) ;
                    break ;
            }
        }
        else {
            switch(mode)
            {
                case Coast:
                    controller_.setIdleMode(IdleMode.kCoast) ;
                    break ;

                case Brake:
                    controller_.setIdleMode(IdleMode.kBrake) ;
                break ;
            }
        }
    }

    public void follow(MotorController ctrl, boolean invert) throws BadMotorRequestException {
        if (sim_ == null) {
            try {
                SparkMaxMotorController other = (SparkMaxMotorController)ctrl ;
                controller_.follow(other.controller_, invert) ;
            }
            catch(ClassCastException ex)
            {
                throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
            }
        }
    }

    public String getType() {
        String ret = null ;

        if (brushless_)
        {
            ret = "SparkMax:brushless" ;
        }
        else
        {
            ret = "SparkMax:brushed" ;
        }

        return ret ;
    }

    public boolean hasPosition() {
        return brushless_ ;
    }

    public double getPosition() throws BadMotorRequestException {
        double ret = 0 ;

        if (!brushless_)
            throw new BadMotorRequestException(this, "brushed motor does not support getPosition()") ;

        if (sim_ != null) {
            ret = sim_encoder_.get() * (double)TicksPerRevolution ;
        } else {
            ret = encoder_.getPosition() * TicksPerRevolution ;
        }

        return ret ;
    }

    public void resetEncoder() throws BadMotorRequestException {
        if (!brushless_)
            throw new BadMotorRequestException(this, "brushed motor does not support getPosition()") ;

        if (sim_ != null) {
            sim_encoder_.set(0.0) ;
        }
        else {
            encoder_.setPosition(0.0) ;
        }
    }

    public void setCurrentLimit(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            controller_.setSmartCurrentLimit((int)limit) ;
        }
    }      

    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            controller_.setOpenLoopRampRate(limit) ;
        }
    } 

    public String getFirmwareVersion() throws BadMotorRequestException {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 24) & 0xff) + "." + String.valueOf((v >> 16) & 0xff) ;
    }
} ;
