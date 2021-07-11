package org.xero1425.simulator.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.XeroMath;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

public class SwerveDriveModel extends SimulationModel {
    // private SwerveDriveModuleModel fl_;
    // private SwerveDriveModuleModel fr_;
    // private SwerveDriveModuleModel bl_;
    // private SwerveDriveModuleModel br_;
    private SwerveDriveModuleModel [] models_ ;
    private NavXModel navx_ ;
    private double [] previous_ ;

    private double xpos_ ;
    private double ypos_ ;
    private double angle_ ;

    //
    // The name of the event that contains the x position of the robot.  This is also the name
    // of the value that is published in the network table for the X position of the robot.
    //
    private final static String XPosName = "xpos" ;

    //
    // The name of the event that contains the y position of the robot.   This is also the name
    // of the value that is published in the network table for the Y position of the robot.
    //
    private final static String YPosName = "ypos" ;

    //
    // The name of the event that contains the angle of the robot.  This is also the name
    // of the value that is published in the network table for the angle of the robot.
    //
    private final static String AngleName = "angle" ;

    private final static String FLAngleName = "fl:angle" ;
    private final static String FRAngleName = "fr:angle" ;
    private final static String BLAngleName = "bl:angle" ;
    private final static String BRAngleName = "br:angle" ;            

    public SwerveDriveModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        xpos_ = 0.0 ;
        ypos_ = 0.0 ;
        angle_ = 0.0 ;

        previous_ = new double[4] ;
        for(int i = 0 ; i < previous_.length ; i++)
            previous_[i] = 0.0 ;
    }

    /// \brief called once at the end of the simulator loop
    @Override
    public void endCycle() {
    }

    /// \brief create a new simulation model for a tank drive.
    /// Model creation is a two step process. The constructor does very basic variable initialization.  This
    /// method completes the model creation process.  This method can rely on all of the other models required being in
    /// place.  However, the create() function on other models may or may not have been called.
    @Override
    public boolean create() {

        if (!attachHardware())
            return false ;
           
        setCreated() ;

        return true ;
    }

    /// \brief run on simlator loop
    /// \param dt the amount of time that has passed since the last simulator loop
    @Override
    public void run(double dt) {
        for(int i = 0 ; i < models_.length ; i++)
            models_[i].run(dt) ;

        double [] angles = new double[4] ;
        double [] deltapos = new double[4] ;        

        for(int i = 0 ; i < 4 ; i++) {
            angles[i] = models_[i].getAngle() ;
            deltapos[i] = models_[i].getPosition() - previous_[i]  ;
            previous_[i] = models_[i].getPosition() ;
        }

        calculatePosition(angles, deltapos) ;

        MessageLogger logger = getEngine().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("SwerveDriveMotorPower: ") ;
        logger.add("flsteer", models_[0].getSteerPower()) ;
        logger.add("fldrive", models_[0].getDrivePower()) ;
        logger.add("frsteer", models_[1].getSteerPower()) ;
        logger.add("frdrive", models_[1].getDrivePower()) ;
        logger.add("blsteer", models_[2].getSteerPower()) ;
        logger.add("bldrive", models_[2].getDrivePower()) ;
        logger.add("brsteer", models_[3].getSteerPower()) ;
        logger.add("brdrive", models_[3].getDrivePower()) ;
        logger.endMessage();
    }

    /// \brief process an event assigned to the subsystem
    /// This subsystem understands the "xpos", "ypos", and "angle" events which
    /// allow the simulation stimulus file to set the position of the robot.  While not
    /// required, these events are usually assigned at time t = 0 to set the initial
    /// position of the robot.  If these events are assigned after t = 0, the position
    /// tracking code in this model may produce strange results.
    /// \param name the name of the event
    /// \param value the value of the event
    public boolean processEvent(String name, SettingsValue value) {
        if (name.equals(XPosName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                xpos_ = value.getDouble();
            } catch (BadParameterTypeException e) {
            }
        }
        else if (name.equals(YPosName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                ypos_ = value.getDouble();
            } catch (BadParameterTypeException e) {
            }
        }  
        else if (name.equals(AngleName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                angle_ = XeroMath.deg2rad(value.getDouble()) ;
            } catch (BadParameterTypeException e) {
            }
        }     
        else if (name.equals(FLAngleName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                models_[0].setAngle(value.getDouble()) ;
            } catch (BadParameterTypeException e) {
            }
        }    
        else if (name.equals(FRAngleName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                models_[1].setAngle(value.getDouble()) ;
            } catch (BadParameterTypeException e) {
            }
        }
        else if (name.equals(BLAngleName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                models_[2].setAngle(value.getDouble()) ;
            } catch (BadParameterTypeException e) {
            }
        }     
        else if (name.equals(BRAngleName)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                models_[3].setAngle(value.getDouble()) ;
            } catch (BadParameterTypeException e) {
            }
        }                    
        return true ;
    }    

    private boolean attachHardware() {
        models_ = new SwerveDriveModuleModel[4] ;
        
        try {
            models_[0] = new SwerveDriveModuleModel(this, "fl") ;
            models_[1] = new SwerveDriveModuleModel(this, "fr") ;
            models_[2] = new SwerveDriveModuleModel(this, "bl") ;
            models_[3] = new SwerveDriveModuleModel(this, "br") ;
        }
        catch(Exception ex) {
            return false ;
        }

        //
        // Attach to the navx model to update the navx angle settings as the robot turns
        //
        if (hasProperty("navx:model") && getProperty("navx:model").isString() && hasProperty("navx:instance")
                && getProperty("navx:instance").isString()) {

            String navx_model = null ;
            String navx_inst = null ;

            try {
                navx_model = getProperty("navx:model").getString();

            } catch (BadParameterTypeException e) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("invalid property types for property 'navx:model'") ;
                logger.endMessage();

                return false ;
            }

            try {
                navx_inst = getProperty("navx:instance").getString();

            } catch (BadParameterTypeException e) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("invalid property types for property 'navx:instance'") ;
                logger.endMessage();

                return false ;
            }

            SimulationModel model = getEngine().findModel(navx_model, navx_inst) ;
            if (model == null || !(model instanceof NavXModel))
            {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("invalid property types for property 'navx:instance'") ;
                logger.endMessage();

                return false ;
            }

            navx_ = (NavXModel)model ;
        }

        return true ;
    }

    private void calculatePosition(double [] angles, double [] deltapos) {
        
    }
}
