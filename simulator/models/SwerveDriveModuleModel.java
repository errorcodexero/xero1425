package org.xero1425.simulator.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;

public class SwerveDriveModuleModel {
    private SimMotorController steer_ ;
    private SimMotorController drive_ ;
    private int encoder_input_ ;
    private double degrees_per_second_per_volt_ ;
    private double angle_ ;
    private double voltage_ ;
    private EncoderMapper mapper_ ;

    public SwerveDriveModuleModel(SwerveDriveModel model, String name) throws Exception {
        double rmax, rmin, emax, emin, rc, ec ;
        String motorname = name + "steer" ;

        angle_ = 0 ;

        steer_ = new SimMotorController(model, motorname) ;
        if (!steer_.createMotor())
        {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("motorname").endMessage();
            throw new Exception("cannot create motor '" + motorname + "' - check log file") ;
        }

        motorname = name + "drive" ;
        drive_ = new SimMotorController(model, motorname) ;
        if (!drive_.createMotor())
        {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("motorname").endMessage();
            throw new Exception("cannot create motor '" + motorname + "' - check log file") ;
        }

        try {
            encoder_input_ = model.getProperty(name + "steer:" + "encoder").getInteger() ;
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("encoder").endMessage();
            throw new Exception("missing encoder property - check log file") ;
        }

        try {
            degrees_per_second_per_volt_ = model.getProperty(name + "steer:" + "degrees_per_second_per_volt").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("ticks_per_second_per_volt").endMessage();
            throw new Exception("missing degrees_per_second_per_volt property - check log file") ;
        }


        try {
            rmax = model.getProperty(name + "steer:" + "rmax").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("rmax").endMessage();
            throw new Exception("missing rmax property - check log file") ;
        }

        try {
            rmin = model.getProperty(name + "steer:" + "rmin").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("rmin").endMessage();
            throw new Exception("missing rmin property - check log file") ;
        }   
            
        try {
            emax = model.getProperty(name + "steer:" + "emax").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("emax").endMessage();
            throw new Exception("missing emax property - check log file") ;
        }
        
        try {
            emin = model.getProperty(name + "steer:" + "emin").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("emin").endMessage();
            throw new Exception("missing emin property - check log file") ;
        }
        
        try {
            rc = model.getProperty(name + "steer:" + "rc").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("rc").endMessage();
            throw new Exception("missing rc property - check log file") ;
        }
        
        try {
            ec = model.getProperty(name + "steer:" + "ec").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = model.getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(model.getModelName()).add(" instance ").addQuoted(model.getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("ec").endMessage();
            throw new Exception("missing rc property - check log file") ;
        }        

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec);        
    }

    public void run(double dt) {
        double power = steer_.getPower() ;
        angle_ += degrees_per_second_per_volt_ * dt * power ;
        voltage_ = mapper_.toEncoder(angle_) ;
        AnalogInDataJNI.setVoltage(encoder_input_, voltage_) ;
    }

    public double getSteerPower() {
        return steer_.getPower() ;
    }

    public double getDrivePower() {
        return drive_.getPower() ;
    }
}
