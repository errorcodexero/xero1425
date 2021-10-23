package org.xero1425.base.motors;

import com.ctre.phoenix.ErrorCode;

/// \file
/// This file conatins the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonFX,
/// the TalonSRX, and the VictorSPX.
///

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \brief This class is MotorController class that supports the TalonFX, TalonSRX, and the VictorSPX motors.
public class TalonFXMotorController extends MotorController
{  
    private TalonFX controller_ ;
    private boolean inverted_ ;
    private boolean pid_setup_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimDouble sim_encoder_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    public final static String SimDeviceName = "TalonFXController" ;
    private final int ControllerTimeout = 100 ;

    public TalonFXMotorController(String name, int index) throws MotorRequestFailedException {
        super(name) ;

        inverted_ = false ;
        pid_setup_ = false ;

        if (RobotBase.isSimulation()) {
            sim_ = SimDevice.create(SimDeviceName, index) ;

            sim_power_ = sim_.createDouble(MotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_encoder_ = sim_.createDouble(MotorController.SimEncoderParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(MotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(MotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;
            sim_.createBoolean(MotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, true) ;

        }
        else {
            ErrorCode code ;

            sim_ = null ;
            sim_power_ = null ;
            sim_encoder_ = null ;

            controller_ = new TalonFX(index) ;

            code = controller_.configFactoryDefault(ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configFactoryDefault() call failed during initialization", code) ;
                
            code = controller_.configVoltageCompSaturation(12.0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configVoltageCompSaturation() call failed during initialization", code) ;

            controller_.enableVoltageCompensation(true);

            code = controller_.setSelectedSensorPosition(0, 0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE setSelectedSensorPosition() call failed during initialization", code) ;

            code = controller_.configNeutralDeadband(0.001, ControllerTimeout);
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configNeutralDeadband() call failed during initialization", code) ;

            code = controller_.configNominalOutputForward(0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configNominalOutputForward() call failed during initialization", code) ;

            code = controller_.configNominalOutputReverse(0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configNominalOutputReverse() call failed during initialization", code) ;

            code = controller_.configPeakOutputForward(1, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configPeakOutputForward() call failed during initialization", code) ;

            code = controller_.configPeakOutputReverse(-1, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configPeakOutputReverse() call failed during initialization", code) ;
                
        }
    }

    public String typeName() {
        return "TalonFX" ;
    }

    public double getInputVoltage() throws BadMotorRequestException {
        return controller_.getBusVoltage() ;
    }

    public double getAppliedVoltage() throws BadMotorRequestException {
        return controller_.getMotorOutputVoltage() ;
    }

    public boolean hasPID() throws BadMotorRequestException {
        return true ;
    }

    public void setTarget(double target) throws BadMotorRequestException {
        if (pid_setup_ == false)
            throw new BadMotorRequestException(this, "calling setTarget() before calling setPID()");

        TalonFX ctrl = (TalonFX)controller_ ;
        ctrl.set(TalonFXControlMode.Velocity, target) ;
    }

    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {
        ErrorCode code ;

        code = controller_.config_kP(0, p, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kP() call failed during setPID() call", code) ; 

        code = controller_.config_kI(0, i, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kI() call failed during setPID() call", code) ; 

        code = controller_.config_kD(0, d, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kD() call failed during setPID() call", code) ; 

        code = controller_.config_kF(0, f, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kF() call failed during setPID() call", code) ;  

        code = controller_.configClosedLoopPeakOutput(0, outmax, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kF() call failed during setPID() call", code) ; 

        pid_setup_= true ;
    }

    public void stopPID() throws BadMotorRequestException {
        controller_.set(ControlMode.PercentOutput, 0.0) ;
    }

    public void setPositionConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        ErrorCode code = controller_.configSelectedFeedbackCoefficient(factor, 0, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE configSelectedFeedbackCoefficient() call failed during setPositionConversion() calls", code) ;         
    }

    public void setVelocityConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        ErrorCode code = controller_.configSelectedFeedbackCoefficient(factor, 0, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE configSelectedFeedbackCoefficient() call failed during setPositionConversion() calls", code) ; 
    }

    public void set(double percent) {
        if (sim_ != null) {
            sim_power_.set(percent) ;
        }
        else {
            controller_.set(ControlMode.PercentOutput, percent) ;
        }
    }

    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(true) ;
        }
        else {
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
        }
        else {
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
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
                    break ;

                case Brake:
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                break ;            
            }
        }
    }

    public void follow(MotorController ctrl, boolean invert) throws BadMotorRequestException {
        if (sim_ == null) {
            if (invert)
                throw new BadMotorRequestException(this, "cannot follow another controller inverted") ;

            try {
                TalonFXMotorController other = (TalonFXMotorController)ctrl ;
                controller_.follow(other.controller_) ;
            }
            catch(ClassCastException ex)
            {
                throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
            }
        }
    }

    public String getType() {
        return "TalonFX" ;
    }

    public boolean hasPosition() {
        return true ;
    }

    public double getPosition() throws BadMotorRequestException {
        double ret = 0 ;

        if (sim_ != null) {
            ret = (int)sim_encoder_.getValue().getDouble() ;
        }
        else {
            TalonFX fx = (TalonFX)controller_ ;
            ret = fx.getSelectedSensorPosition() ;
        }
        
        return ret ;
    }

    public void resetEncoder() throws BadMotorRequestException {
        if (sim_ != null) {
            sim_encoder_.set(0.0) ;
        }
        else {
            TalonFX fx = (TalonFX)controller_ ;
            fx.setSelectedSensorPosition(0) ;
        }
    }

    public void setCurrentLimit(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            TalonFX fx = (TalonFX)controller_ ;
            SupplyCurrentLimitConfiguration cfg = new SupplyCurrentLimitConfiguration(true, limit, limit, 1) ;
            fx.configSupplyCurrentLimit(cfg) ;
        }
    }     
    
    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            TalonFX fx = (TalonFX)controller_ ;
            fx.configOpenloopRamp(limit, 20) ;
        }
    }  

    public String getFirmwareVersion() throws BadMotorRequestException {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;
    }

    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) throws BadMotorRequestException {
        if (controller_ != null)
        {
            if (freq == EncoderUpdateFrequency.Infrequent) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 500) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 500) ;
            }
            else if (freq == EncoderUpdateFrequency.Default) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20) ;            
            }
            else if (freq == EncoderUpdateFrequency.Frequent) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10) ;             
            }
        }        
    }     
} ;
