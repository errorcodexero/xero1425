package org.xero1425.base.swervedrive;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.SwerveDriveGamepad;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsParser;
import org.xero1425.misc.Speedometer;

/// \brief The swerve drive subsystem for driving a robot using a drive base where each wheel can be independently steered.
public class SwerveDriveSubsystem extends DriveBaseSubsystem {
    //
    // This object stores the names of the swerve modules
    //
    public class Names {
        public Names(String sname, String lname) {
            ShortName = sname ;
            LongName = lname ;
        }

        public String ShortName ;
        public String LongName ;
    } ;

    private double width_;                                                                      // The width of the robot, from the settings file
    private double length_;                                                                     // The length of the robot, from the settings file
    private SwerveModule[] pairs_;                                                              // The serve modules, created by this class
    private Speedometer angular_;                                                               // The angular position, velocity, and acceleration of the robot
    private double rotate_angle_ ;                                                              // The base angle for a rotate vector, calculated from the length and width
    private double maxspeed_ ;                                                                  // The maximum linear speed of the 

    static public final int FL = 0;                                                             // Index of the front left module
    static public final int FR = 1;                                                             // Index of the front right module
    static public final int BL = 2;                                                             // Index of the back left module
    static public final int BR = 3;                                                             // Index of the back right module

    static private Names[] names_ = new Names[4] ;                                              // The names of each module (short name and long name)

    static private final String AngularSamplesName = "swervedrive:angularsamples";              // The settings file entry for the angular speedometer


    /// \brief create the serve drive subsystem
    /// \param parent the parent subsystem
    /// \param name the name of the subsystem
    /// \param config the prefix for configuration entries in the settings file
    public SwerveDriveSubsystem(Subsystem parent, String name, String config) throws Exception {
        super(parent, name);

        // Set the module names (short and long)
        names_[FL] = new Names("fl",  "FrontLeft") ;
        names_[FR] = new Names("fr", "FrontRight") ;
        names_[BL] = new Names("bl", "BackLeft") ;
        names_[BR] = new Names("br", "BackRight") ;

        //
        // Get the parameters from the settings file
        //
        SettingsParser settings = getRobot().getSettingsParser();
        width_ = settings.get("swervedrive:width").getDouble();
        length_ = settings.get("swervedrive:length").getDouble();
        maxspeed_ = settings.get("swervedrive:maxspeed").getDouble();

        //
        // Create the swerve modules
        //
        pairs_ = new SwerveModule[getModuleCount()];
        for (int i = 0; i < getModuleCount(); i++) {
            pairs_[i] = new SwerveModule(getRobot(), this, names_[i].LongName, config + ":" + names_[i].ShortName);
        }

        //
        // Create the angular speedometer to track angular position, velocity, and acceleration
        //
        int angularsamples = 2 ;
        if (settings.isDefined(AngularSamplesName) && settings.get(AngularSamplesName).isInteger()) {
            angularsamples = settings.get(AngularSamplesName).getInteger();
        }
        angular_ = new Speedometer("angles", angularsamples, true);

        //
        // Calculate the angle for the velocity vector to rotate the robot
        //
        rotate_angle_ = Math.toDegrees(Math.atan2(length_, width_)) ;

        //
        // Reset the GYRO to zero degrees
        //
        gyro().reset() ;
    }

    public double getMaxSpeed() {
        return maxspeed_ ;
    }

    /// \brief return the base angle for a vector to rotate the robot.
    /// \returns the base angle for a vector to rotate the robot
    public double getPHI() {
        return rotate_angle_ ;
    }

    public int getModuleCount() {
        return names_.length ;
    }

    public SwerveModule getModule(int index) {
        return pairs_[index] ;
    }

    public double getWidth() {
        return width_;
    }

    public double getLength() {
        return length_;
    }

    /// \brief returns the current angle in degrees of the robot
    /// \returns the current angle of the robot
    public double getAngle() {
        return angular_.getDistance() ;
    }

    public double getModuleAngle(int module) {
        return getModule(module).getAngle() ;
    }

    public double getAcceleration() {
        double total = 0.0 ;

        for(int i = 0 ; i < getModuleCount() ; i++) {
            total += getModule(i).getAcceleration() ;
        }

        return total / getModuleCount() ;
    }

    public double getVelocity() {
        double total = 0.0 ;

        for(int i = 0 ; i < getModuleCount() ; i++) {
            total += getModule(i).getVelocity() ;
        }

        return total / getModuleCount() ;
    }

    public double getDistance() {
        double total = 0.0 ;

        for(int i = 0 ; i < getModuleCount() ; i++) {
            total += getModule(i).getDistance() ;
        }

        return total / getModuleCount() ;
    }

    public Gamepad createGamePad(OISubsystem oi, int index, DriveBaseSubsystem drive) throws Exception {
        return new SwerveDriveGamepad(oi, index, drive);
    }

    /// \brief returns true to indicate this is a drivebase
    /// \returns true to indicate this is a drivebase
    public boolean isDB() {
        return true;
    }

    /// \brief This method is called when the robot enters one of its specifc modes.
    /// The modes are Autonomous, Teleop, Test, or Disabled. It is used to set the
    /// neutral mode specifically for the robot mode.
    public void init(LoopType ltype) {
        super.init(ltype);

        MotorController.NeutralMode nm = loopTypeToNeutralMode(ltype);
        for (int i = 0; i  < getModuleCount() ; i++) {
            try {
                getModule(i).setNeutralMode(nm);
            } catch (Exception ex) {
            }
        }
    }

    public String getPairName(int which, boolean sname) {

        return sname ? names_[which].ShortName : names_[which].LongName ;
    }

    public void computeMyState() throws BadMotorRequestException {
        //
        //
        for (int i = 0; i < getModuleCount(); i++)
            getModule(i).computeMyState(getRobot().getDeltaTime());

        if (gyro() != null) {
            double angle = gyro().getYaw();
            angular_.update(getRobot().getDeltaTime(), angle);
        }

        putDashboard("fl", DisplayType.Always, getModule(FL).status());
        putDashboard("fr", DisplayType.Always, getModule(FR).status());
        putDashboard("bl", DisplayType.Always, getModule(BL).status());
        putDashboard("br", DisplayType.Always, getModule(BR).status());

        putDashboard("flticks", DisplayType.Verbose, getModule(FL).getTicks()) ;
        putDashboard("frticks", DisplayType.Verbose, getModule(FR).getTicks()) ;
        putDashboard("blticks", DisplayType.Verbose, getModule(BL).getTicks()) ;
        putDashboard("brticks", DisplayType.Verbose, getModule(BR).getTicks()) ;

        putDashboard("dbangle", DisplayType.Always, getAngle()) ;

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("angle", getAngle()) ;
        logger.add("fl", getModule(FL).status()) ;
        logger.add("fr", getModule(FR).status()) ;
        logger.add("bl", getModule(BL).status()) ;        
        logger.add("br", getModule(BR).status()) ;
        logger.endMessage();        
    }

    @Override
    public void run() throws Exception {
        super.run();

        double dt = getRobot().getDeltaTime();
        for (int i = 0; i < getModuleCount(); i++)
            getModule(i).run(dt);

    }

    public void stop() throws BadMotorRequestException, MotorRequestFailedException {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            getModule(i).set(0.0, 0.0) ;
        }
    }

    public void setPower(int which, double steer, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        getModule(which).set(steer, drive) ;
    }

    
    public void setSteerMotorPower(int which, double steer) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        getModule(which).setSteerMotorPower(steer) ;
    }

    
    public void setDriveMotorPower(int which, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        getModule(which).setDriveMotorPower(drive) ;
    }

    public void setTargets(double[] angles, double[] speeds) {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            getModule(i).setTargetAngle(angles[i]);
            getModule(i).setTargetVelocity(speeds[i]) ;
        }
    }

    public void setAngle(double angle) {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            getModule(i).setTargetAngle(angle);
        }        
    }

    public void setNoTargets() {
        for(int i = 0 ; i <= getModuleCount() ; i++) {
            getModule(i).setNoAngle();
            getModule(i).setNoSpeed();
        }
    }
}
