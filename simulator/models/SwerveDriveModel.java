package org.xero1425.simulator.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SwerveDriveModuleModel;

import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

public class SwerveDriveModel extends SimulationModel {
    private SwerveDriveModuleModel fl_;
    private SwerveDriveModuleModel fr_;
    private SwerveDriveModuleModel bl_;
    private SwerveDriveModuleModel br_;
    private NavXModel navx_ ;
    private SwerveDrivePoseEstimator estimator_;

    public SwerveDriveModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        // Rotation2d gyroAngle = Rotation2d.fromDegrees(0.0) ;
        // Pose2d initialPoseMeters = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)) ;


        // Translation2d fl = new Translation2d() ;
        // SwerveDriveKinematics km = new SwerveDriveKinematics() ;

        // estimator_ = new SwerveDrivePoseEstimator(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs) ;
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
        fl_.run(dt) ;
        fr_.run(dt) ;
        bl_.run(dt) ;
        br_.run(dt) ;

        MessageLogger logger = getEngine().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("SwerveDriveMotorPower: ") ;
        logger.add("flsteer", fl_.getSteerPower()) ;
        logger.add("fldrive", fl_.getDrivePower()) ;
        logger.add("frsteer", fr_.getSteerPower()) ;
        logger.add("frdrive", fr_.getDrivePower()) ;
        logger.add("blsteer", bl_.getSteerPower()) ;
        logger.add("bldrive", bl_.getDrivePower()) ;
        logger.add("brsteer", br_.getSteerPower()) ;
        logger.add("brdrive", br_.getDrivePower()) ;
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
        return true ;
    }    

    private boolean attachHardware() {

        try {
            fl_ = new SwerveDriveModuleModel(this, "fl") ;
            fr_ = new SwerveDriveModuleModel(this, "fr") ;
            bl_ = new SwerveDriveModuleModel(this, "bl") ;
            br_ = new SwerveDriveModuleModel(this, "br") ;
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

    private double inchesToMeters(double in) {
        return in * 0.0254 ;
    }
}
