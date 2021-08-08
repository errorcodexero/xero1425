package org.xero1425.base.oi;

import edu.wpi.first.wpilibj.DriverStation;
import org.xero1425.base.LoopType;
import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.tankdrive.SwerveDriveSubsystem;
import org.xero1425.base.tankdrive.SwerveDirectionRotationAction; 
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsParser;

/// \brief This class controls interprets the input from the game pad to control the swerve drivebase.

public class SwerveDriveGamepad extends Gamepad {
    
    private SwerveDriveSubsystem db_ ;              //TBD create a swerve drive subsystem

    private double power_ ;
    private double max_pos_ ;                       // max. speed of robot [inches/sec]
    private double max_angle_ ;                     // max rotation of robot [degrees/sec] 

    private double deadzone_x_ ;
    private double deadzone_y_ ;
    private double deadzone_rotate_ ;
    
    private SwerveDirectionRotationAction action_ ;  //TBD create a swerve dir rot action


    public SwerveDriveGamepad(OISubsystem oi, int index, SwerveDriveSubsystem drive_) throws Exception {
        super(oi, "Xero1425GamePad", index) ;

        // TBD chack that the drivebase is, indeed, the right kind of db & that the 2 joysticks work appropriately

        db_ = drive_ ;

    }

    @Override
    public void init(LoopType ltype) {
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        SettingsParser settings = getSubsystem().getRobot().getSettingsParser() ;

        //I got the param names from the existing swerve params file bc of consistency, etc. 
        //The only difference is that it's called "deadzone" instead of deadband

        max_pos_ = settings.get("driver:power:max").getDouble() ;
        max_angle_ = settings.get("driver:angle:max").getDouble() ;

        deadzone_x_ = settings.get("driver:position:x:deadzone").getDouble() ;
        deadzone_y_ = settings.get("driver:position:y:deadzone").getDouble() ;
        deadzone_rotate_ = settings.get("driver:rotation:deadzone").getDouble() ;
        
    }

    @Override
    public void computeState() {
        super.computeState() ;
    }

    @Override
    public void generateActions(SequenceAction seq) {
        if (db_ == null || !isEnabled())
            return ;

        //QUESTION: should we make the following a try-catch like in the tankdrivegamepad?

        // TBD
        // get the current values from the gamepad
        // assign them to doubles, which correspond with each of the rotation; pos-y, and pos-x
        // run constraints on the rotation; pos-y; pos-x -- like deadband, etc
        // final-vector calculations... 
        // set the vector (dir + mag[vel]) action to the db

    }

    
    //// QUESTION: Is this (below) the power-scaling factor [I got it from the existing swerve]? Would it be more effective if we scale the raw joystick values before vector calculations?
    
    ///TBD the following function on my own...
    // private double mapJoyStick(double v, double maxv, double db, double power) {
    //     if (Math.abs(v) < db)
    //         return 0.0 ;

    //     return Math.signum(v) * Math.pow(Math.abs(v), power) * maxv ;
    // }

}