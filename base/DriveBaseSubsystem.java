package org.xero1425.base;

import org.xero1425.base.gyro.RomiGyro;

import com.ctre.phoenix.motorcontrol.MotorCommutation;

import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.oi.Gamepad;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;


//#import stuff

public abstract class DriveBaseSubsystem extends Subsystem {
    
    private MotorController.NeutralMode automode_neutral_;
    private MotorController.NeutralMode teleop_neutral_;
    private MotorController.NeutralMode disabled_neutral_;

    private XeroGyro gyro_;


    public DriveBaseSubsystem(Subsystem parent, String name) 
            throws BadParameterTypeException, MissingParameterException {
        super(parent, name) ;

        MessageLogger logger = getRobot().getMessageLogger() ;  
                            
        automode_neutral_ = MotorController.NeutralMode.Brake ;
        teleop_neutral_ = MotorController.NeutralMode.Brake ;
        disabled_neutral_ = MotorController.NeutralMode.Coast ;

        String typeofgyro = getRobot().getSettingsParser().get("hw:gyro").getString() ;
        if (typeofgyro.equals("navx")) {
            gyro_ = new NavxGyro() ;
        }
        else if (typeofgyro.equals("LSM6DS33")) {
            gyro_ = new RomiGyro() ;
        }
        else {          //I added this, for the case that there is no gyro specified, etc
            logger.startMessage(MessageType.Error) ;
            logger.add("gyro type not correctly specified in .dat file") ;
            logger.endMessage() ;
        }

        double start = getRobot().getTime() ;

        while((getRobot().getTime() - start) < 3) {
            if (gyro_.isConnected()) {      /// This was a bug in the original version: had "gyro()" instead of "gyro_"
                break ;
            }
        }
        if (!gyro_.isConnected()) {
            logger.startMessage(MessageType.Error) ;
            logger.add("gyro not connected") ;
            logger.endMessage() ;
            gyro_ = null ;                  
        }

    }

    //Gamepad
    abstract void createGamePad() ;

    //DB
    public boolean isDriveBase() {
        return true ;
    }

    //Gyro
    protected boolean HasGyro() {
        return gyro_ != null ;
    }
    protected XeroGyro gyro() {
        return gyro_ ;
    }

    //Neutral Modes
    protected MotorController.NeutralMode autoModeNeutral() {
        return automode_neutral_ ;
    }
    protected MotorController.NeutralMode teleopModeNeutral() {
        return teleop_neutral_ ;
    }
    protected MotorController.NeutralMode disabledModeNeutral() {
        return disabled_neutral_ ;
    }

    protected MotorController.NeutralMode loopTypeToNeutralMode(LoopType ltype) {
        MotorController.NeutralMode nm = MotorController.NeutralMode.Brake;

        switch(ltype) {
            case Autonomous:
                nm = automode_neutral_ ;
                break ;
            case Teleop:
                nm = teleop_neutral_ ;
                break ;
            case Test:
                nm = disabled_neutral_ ;
                break ;
            case Disabled:
                nm = disabled_neutral_ ;
                break ;   
        }
        return nm ;
    }

    /// TBD add common interface:
    // pos
    // vel
    // accel
    // heading


}