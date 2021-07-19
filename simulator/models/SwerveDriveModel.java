package org.xero1425.simulator.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveDriveModel extends SimulationModel {
    private SwerveDriveModuleModel [] models_ ;
    private NavXModel navx_ ;
    private double [] previous_ ;
    private double elapsed_ ;

    private double xpos_ ;
    private double ypos_ ;
    private double angle_ ;

    private double width_ ;
    private double length_ ;
    private SwerveDriveKinematics kinematics_ ;
    private SwerveDriveOdometry odometry_ ;

    private int module_logger_id_ ;

    //
    // The network table name for publising information about tank drive in the network tables
    //
    private final static String SubTableName = "tankdrive" ;

    //
    // The name of the event that contains the x position of the robot.  This is also the name
    // of the value that is published in the network table for the X position of the robot.
    //
    private final static String TankDriveXPos = "xpos" ;

    //
    // The name of the event that contains the y position of the robot.   This is also the name
    // of the value that is published in the network table for the Y position of the robot.
    //
    private final static String TankDriveYPos = "ypos" ;

    //
    // The name of the event that contains the angle of the robot.  This is also the name
    // of the value that is published in the network table for the angle of the robot.
    //
    private final static String TankDriveAngle = "angle" ;

    private final static String FLAngleName = "fl:angle" ;
    private final static String FRAngleName = "fr:angle" ;
    private final static String BLAngleName = "bl:angle" ;
    private final static String BRAngleName = "br:angle" ;            

    public SwerveDriveModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        module_logger_id_ = engine.getMessageLogger().registerSubsystem("sweverdrive_module_model") ;

        previous_ = new double[4] ;
        for(int i = 0 ; i < previous_.length ; i++)
            previous_[i] = 0.0 ;

        elapsed_ = 0.0 ;

        angle_ = 0.0 ;
        xpos_ = 0.0 ;
        ypos_ = 0.0 ;
    }

    public int getModuleLoggerID() {
        return module_logger_id_ ;
    }

    /// \brief called once at the end of the simulator loop
    @Override
    public void endCycle() {
        //
        // Write information to the log file about the position of the robot as seen by the model
        //
        MessageLogger logger = getEngine().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("swervedrive") ;
        logger.add(" ").add(xpos_) ;
        logger.add(" ").add(ypos_) ;
        logger.add(" ").add(angle_) ;
        logger.endMessage() ;

        //
        // Write information to the network tables about the position on the robot
        //
        NetworkTable table_ = NetworkTableInstance.getDefault().getTable(SimulationEngine.NetworkTableName).getSubTable(SubTableName) ;
        table_.getEntry(TankDriveXPos).setNumber(xpos_) ;
        table_.getEntry(TankDriveYPos).setNumber(ypos_) ;
        table_.getEntry(TankDriveAngle).setNumber(angle_) ;
    }

    /// \brief create a new simulation model for a tank drive.
    /// Model creation is a two step process. The constructor does very basic variable initialization.  This
    /// method completes the model creation process.  This method can rely on all of the other models required being in
    /// place.  However, the create() function on other models may or may not have been called.
    @Override
    public boolean create() {
        double angle = 0.0 ;     

        if (!attachHardware())
        {
            return false ;
        }
        
        try {
            width_ = getProperty("width").getDouble() ;
            length_ = getProperty("length").getDouble();

            if (hasProperty("angle")) {
                angle = getProperty("angle").getDouble() ;
            }
        }
        catch(Exception ex) {
            return false ;
        }

        double w = inches2Meters(width_) ;
        double l = inches2Meters(length_) ;
        Translation2d fl = new Translation2d(-w / 2.0, l / 2.0) ;
        Translation2d fr = new Translation2d(w / 2.0, l / 2.0) ;
        Translation2d bl = new Translation2d(-w / 2.0, -l / 2.0) ;
        Translation2d br = new Translation2d(w / 2.0, -l / 2.0) ;

        kinematics_ = new SwerveDriveKinematics(fl, fr, bl, br) ;
        odometry_ = new SwerveDriveOdometry(kinematics_, Rotation2d.fromDegrees(angle));
           
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

        calculatePosition(dt, angles, deltapos) ;

        MessageLogger logger = getEngine().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getModuleLoggerID()) ;
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
        if (name.equals(FLAngleName)) {
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

    private void calculatePosition(double dt, double [] angles, double [] deltapos) {
        //
        // This does not, and may never work.  It relies on the robot angle as an input
        // from the GYRO and we want to calculate the angle of the GYRO using this.  We also
        // need the ground speed of the wheels in meters per second, so we need to do some 
        // math there to take the delta time and delta pos to get velocity in m/s.
        //

        elapsed_ += dt ;

        SwerveModuleState fl  = new SwerveModuleState(deltapos[0] / dt, Rotation2d.fromDegrees(angles[0])) ;
        SwerveModuleState fr  = new SwerveModuleState(deltapos[1] / dt, Rotation2d.fromDegrees(angles[1])) ;
        SwerveModuleState bl  = new SwerveModuleState(deltapos[2] / dt, Rotation2d.fromDegrees(angles[2])) ;
        SwerveModuleState br  = new SwerveModuleState(deltapos[3] / dt, Rotation2d.fromDegrees(angles[3])) ;
        odometry_.updateWithTime(elapsed_, Rotation2d.fromDegrees(angle_), fl, fr, bl, br) ;

        Pose2d pose = odometry_.getPoseMeters() ;
        navx_.setYaw(pose.getRotation().getDegrees()) ;

        xpos_ = meters2Inches(pose.getTranslation().getX()) ;
        ypos_ = meters2Inches(pose.getTranslation().getY()) ;
    }

    static final double InchesPerMeter = 0.0254 ;

    private double inches2Meters(double in) {
        return in * InchesPerMeter ;
    }

    private double meters2Inches(double mtr) {
        return mtr / InchesPerMeter ;
    }
}
