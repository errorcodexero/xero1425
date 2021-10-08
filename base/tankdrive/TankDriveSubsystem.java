package org.xero1425.base.tankdrive;

import java.util.Map;
import java.util.HashMap;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

import org.xero1425.base.LoopType;
import org.xero1425.base.PositionTracker;
import org.xero1425.base.Subsystem;
import org.xero1425.base.gyro.RomiGyro;
import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.EncoderUpdateFrequency;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsParser;
import org.xero1425.misc.Speedometer;

/// \brief The tankdrive subsystem for driving a robot using a tank drive type drivebase.
/// 
/// This class expects entries in the settings file.
///
///      hw:tankdrive:motors:left:type                                           "romi"
///      hw:tankdrive:motors:left:canid                                          0
///      hw:tankdrive:encoder:left:1                                             4
///      hw:tankdrive:encoder:left:2                                             5
///      hw:tankdrive:motors:right:type                                          "romi"
///      hw:tankdrive:motors:right:canid                                         1
///      hw:tankdrive:motors:right:inverted                                      true
///      hw:tankdrive:encoder:right:1                                            6
///      hw:tankdrive:encoder:right:2                                            7
///      
///      hw:tankdrive:gyro                                                       "LSM6DS33"
///
///      tankdrive:inches_per_tick                                               0.00601246292357961603042471078223
///      tankdrive:width                                                         5.55
///      tankdrive:scrub                                                         1.0
///      tankdrive:speedometer:linearsamples                                     4
///      tankdrive:speedometer:angularsamples                                    2
///
///      tankdrive:follower:left:ka                                              0.002624471
///      tankdrive:follower:left:kv                                              0.02637896
///      tankdrive:follower:left:kp                                              0.1
///      tankdrive:follower:left:kd                                              0.0
///
///      tankdrive:follower:right:ka                                             0.002624471
///      tankdrive:follower:right:kv                                             0.02637896
///      tankdrive:follower:right:kp                                             0.1
///      tankdrive:follower:right:kd                                             0.0
///
///      tankdrive:follower:angle_correction                                     0.0
///
public class TankDriveSubsystem extends Subsystem {

    private PositionTracker tracker_ ;
    private double left_power_ ;
    private double right_power_ ;
    private int ticks_left_ ;
    private int ticks_right_ ;
    private double dist_l_ ;
    private double dist_r_ ;
    private double last_dist_l_ ;
    private double last_dist_r_ ;
    private double left_inches_per_tick_ ;
    private double right_inches_per_tick_ ;
    private double total_angle_ ;
    private XeroGyro gyro_ ;
    private MotorController.NeutralMode automode_neutral_ ;
    private MotorController.NeutralMode teleop_neutral_ ;
    private MotorController.NeutralMode disabled_neutral_ ;

    private Speedometer angular_ ;
    private Speedometer left_linear_ ;
    private Speedometer right_linear_ ;

    private MotorController left_motors_ ;
    private MotorController right_motors_ ;
    private Encoder left_encoder_ ;
    private Encoder right_encoder_ ;

    private boolean recording_ ;
    private double recording_start_ ;

    private Map<String, Double> trips_ ;

    private final static double kEpsilon = 1e-9 ;

    /// \brief create a new tankdrive subsystem
    /// \param parent the parent subsystem
    /// \param name the name of the subsystem
    /// \param config the string prefix to use when searching for settings file entries
    public TankDriveSubsystem(Subsystem parent, String name, String config)
            throws BadParameterTypeException, MissingParameterException, BadMotorRequestException {
        super(parent, name);

        MessageLogger logger = getRobot().getMessageLogger();
        SettingsParser settings = getRobot().getSettingsParser() ;

        recording_ = false ;

        double width = settings.get("tankdrive:width").getDouble() ;
        double scrub = settings.get("tankdrive:scrub").getDouble() ;
        tracker_ = new PositionTracker(width, scrub) ;

        dist_l_ = 0.0;
        dist_r_ = 0.0;
        last_dist_l_ = 0.0 ;
        last_dist_r_ = 0.0 ;

        left_inches_per_tick_ = getRobot().getSettingsParser().get("tankdrive:inches_per_tick").getDouble();
        right_inches_per_tick_ = left_inches_per_tick_;

        int linearsamples = 2 ;
        int angularsamples = 2 ;
        String linear = "tankdrive:speedometer:linearsamples" ;
        String angular = "tankdrive:speedometer:angularsamples" ;

        if (settings.isDefined(linear) && settings.get(linear).isInteger()) {
            linearsamples = settings.get(linear).getInteger() ;
        }

        if (settings.isDefined(angular) && settings.get(angular).isInteger()) {
            angularsamples = settings.get(angular).getInteger() ;
        }

        angular_ = new Speedometer("angles", angularsamples, true);
        left_linear_ = new Speedometer("left", linearsamples, false);
        right_linear_ = new Speedometer("right", linearsamples, false);

        automode_neutral_ = MotorController.NeutralMode.Brake;
        teleop_neutral_ = MotorController.NeutralMode.Brake;
        disabled_neutral_ = MotorController.NeutralMode.Coast;

        String gyrotype = getRobot().getSettingsParser().get("hw:tankdrive:gyro").getString() ;
        if (gyrotype.equals("navx")) {
            gyro_ = new NavxGyro() ;
        }
        else if (gyrotype.equals("LSM6DS33")) {
            gyro_ = new RomiGyro() ;
        }

        double start = getRobot().getTime() ;
        while (getRobot().getTime() - start < 3.0) {
            if (gyro_.isConnected())
                break ;
        }

        if (!gyro_.isConnected()) {
            logger.startMessage(MessageType.Error);
            logger.add("NavX is not connected - cannot perform tankdrive path following functions");
            logger.endMessage();
            gyro_ = null;
        }

        trips_ = new HashMap<String, Double>();

        attachHardware();
    }

    /// \brief sets the recording flag for the drive base.  
    /// When the recording flag is set, the drivebase subsystem writes the pose for the drivebase
    /// into the network tables at the location /Smartdashboard/db-trk-t, /Smartdashboard/db-trk-x, 
    /// /Smartdashboard/db-trk-y,and /Smartdashboard/db-trk-a.  The time is relative to when setRecording
    /// was called with a value of true.  The angle is in degrees.
    public void setRecording(boolean v) {
        recording_ = v ;
        recording_start_ = getRobot().getTime() ;
    }

    /// \brief returns true to indicate this is a drivebase
    /// \returns true to indicate this is a drivebase
    public boolean isDB() {
        return true ;
    }

    /// \brief returns the width of the robot
    /// This width is the track width which is basically from the center of the left 
    /// wheels to the center of the right wheels.
    /// \returns the width of the robot
    public double getWidth() {
        return tracker_.getWidth() ;
    }

    /// \brief returns the scrub value for the robot
    /// \returns the scrub value for the robot
    public double getScrub() {
        return tracker_.getScrub() ;
    }

    /// \brief start a new trip with the given name
    /// A trip measures the distance traveled since a given point in time.  The trip is named and
    /// the name is used to reference the given trip.  This method starts a trip with the given name.
    /// The name can be used in a call to getTripDistance() to see how far the robot has traveled since
    /// the trip was started.
    /// \param name the name of the trip
    public void startTrip(String name) {
        trips_.put(name, getDistance());
    }

    /// \brief returns the distance traveled by the robot since the trip given was started
    /// \param name the name of the trip of interest
    /// \sa startTrip
    /// \returns the distance traveled by the left sdie of the robot 
    public double getTripDistance(String name) {
        if (!trips_.containsKey(name))
            return 0.0;

        return trips_.get(name);
    }

    /// \brief returns the distance traveled by the left sdie of the robot
    /// \returns the distance traveled by the left sdie of the robot     
    public double getLeftDistance() {
        return dist_l_;
    }

    /// \brief returns the distance traveled by the right sdie of the robot
    /// \returns the distance traveled by the right sdie of the robot 
    public double getRightDistance() {
        return dist_r_;
    }

    /// \brief returns the distance traveled by the robot
    /// \returns the distance traveled by the robot
    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    /// \brief returns the velocity of the left side of the robot
    /// \returns the velocity of the left side of the robot    
    public double getLeftVelocity() {
        return left_linear_.getVelocity() ;
    }

    /// \brief returns the velocity of the right side of the robot
    /// \returns the velocity of the right side of the robot    
    public double getRightVelocity() {
        return right_linear_.getVelocity() ;
    }

    /// \brief returns the velocity of the robot
    /// \returns the velocity of the robot
    public double getVelocity() {
        return (left_linear_.getVelocity() + right_linear_.getVelocity()) / 2.0 ;
    }

    /// \brief returns the acceleration of the left side of the robot
    /// \returns the acceleration of the left side of the robot
    public double getLeftAcceleration() {
        return left_linear_.getAcceleration() ;
    }

    /// \brief returns the acceleration of the right side of the robot
    /// \returns the acceleration of the right side of the robot
    public double getRightAcceleration() {
        return right_linear_.getAcceleration() ;
    }    

    /// \brief returns the acceleration of the robot
    /// \returns the acceleration of the robot
    public double getAcceleration() {
        return (left_linear_.getAcceleration() + right_linear_.getAcceleration()) / 2.0 ;
    }

    /// \brief returns the left side ticks value from the encoder
    /// \returns the left side ticks value from the encoder
    public int getLeftTick() {
        return ticks_left_ ;
    }

    /// \brief returns the right side ticks value from the encoder
    /// \returns the right side ticks value from the encoder
    public int getRightTick() {
        return ticks_right_ ;
    }

    /// \brief returns the current angle in degrees of the robot
    /// \returns the current angle of the robot
    public double getAngle() {
        return angular_.getDistance() ;
    }

    /// \brief returns the net total angle in degrees the robot has rotated since the drivebase was initialization
    /// If the robot has rotated in place two complete revolutions, this would return 720 degrees
    /// \returns the net total angle the robot has rotated since the drivebase was initialization
    public double getTotalAngle() {
        return total_angle_ ;
    }

    /// \brief This method is called when the robot enters the disabled state.
    /// It is used in this subsystem to set the neutral mode of the drivebase
    /// motors.
    public void reset() {
        super.reset();

        try {
            left_motors_.setNeutralMode(disabled_neutral_);
            right_motors_.setNeutralMode(disabled_neutral_);
        } catch (Exception ex) {
        }
    }

    /// \brief This method is called when the robot enters one of its specifc modes.
    /// The modes are Autonomous, Teleop, Test, or Disabled.  It is used to set the
    /// neutral mode specifically for the robot mode.
    public void init(LoopType ltype) {
        super.init(ltype);

        try {
            switch (ltype) {
            case Autonomous:
                left_motors_.setNeutralMode(automode_neutral_);
                right_motors_.setNeutralMode(automode_neutral_);
                //left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent);
                //right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent);
                break;

            case Teleop:
                left_motors_.setNeutralMode(teleop_neutral_);
                right_motors_.setNeutralMode(teleop_neutral_);
                //left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent);
                //right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent);
                break;

            case Test:
                left_motors_.setNeutralMode(disabled_neutral_);
                right_motors_.setNeutralMode(disabled_neutral_);
                break;

            case Disabled:
                left_motors_.setNeutralMode(disabled_neutral_);
                right_motors_.setNeutralMode(disabled_neutral_);      
                //left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent);
                //right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent);                      
                break ;
            }
        } catch (Exception ex) {
        }
    }

    /// \brief set the pose (x and y location plus heading) of the robot
    /// \param pose the pose for the robot
    public void setPose(Pose2d pose) {
        tracker_.setPose(pose);
    }

    /// \brief get the pose for the robot
    /// This method returns the current pose based on the position tracker
    /// assigned to the drivebase.
    /// \returns the current pose for the robot
    public Pose2d getPose() {
        return tracker_.getPose() ;
    }

    /// \brief compute the state of the drivebase.
    /// This method reads the encoders for the left and right sides of the drivebase
    /// and reads the gyro as well.  These three are used to compute the current state
    /// of the robot and to update the position tracker for the robot.
    public void computeMyState() {
        double angle = 0.0;

        try {
            if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
                ticks_left_ = (int)left_motors_.getPosition();
                ticks_right_ = (int)right_motors_.getPosition();
            }
            else {
                ticks_left_ = left_encoder_.get() ;
                ticks_right_ = right_encoder_.get()  ;
            }

            dist_l_ = ticks_left_ * left_inches_per_tick_;
            dist_r_ = ticks_right_ * right_inches_per_tick_;
            if (gyro_ != null) {
                angle = gyro_.getYaw();
                angular_.update(getRobot().getDeltaTime(), angle);
            }

            tracker_.updatePosition(dist_l_ - last_dist_l_, dist_r_ - last_dist_r_, angle);
            left_linear_.update(getRobot().getDeltaTime(), getLeftDistance());
            right_linear_.update(getRobot().getDeltaTime(), getRightDistance());

            total_angle_ = gyro_.getAngle() ;

            last_dist_l_ = dist_l_ ;
            last_dist_r_ = dist_r_ ;

            putDashboard("ldist", DisplayType.Verbose, dist_l_);
            putDashboard("rdist", DisplayType.Verbose, dist_r_);

        } catch (Exception ex) {
            //
            // This should never happen
            //
        }

        if (recording_) {
            putDashboard("db-trk-t", DisplayType.Verbose, getRobot().getTime() - recording_start_) ;
            putDashboard("db-trk-x", DisplayType.Verbose, tracker_.getPose().getX());
            putDashboard("db-trk-y", DisplayType.Verbose, tracker_.getPose().getY()) ;
            putDashboard("db-trk-a", DisplayType.Verbose, tracker_.getPose().getRotation().getDegrees());
        }
    }

    
    public TankDriveVelocities inverseKinematics(Twist2d velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new TankDriveVelocities(velocity.dx, velocity.dx);
        }
        double delta_v = getWidth() * velocity.dtheta / (2 * getScrub());
        return new TankDriveVelocities(velocity.dx - delta_v, velocity.dx + delta_v);
    }

    /// \brief set the power for the tank drive
    protected void setPower(double left, double right) {
        left_power_ = left ;
        right_power_ = right ;

        try {
            left_motors_.set(left_power_) ;
            right_motors_.set(right_power_) ;
        }
        catch(BadMotorRequestException|MotorRequestFailedException ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("subsystem ").addQuoted(getName()).add(": cannot set power -").add(ex.getMessage()).endMessage();
        }
    }

    private void attachHardware() throws BadMotorRequestException, MissingParameterException, BadParameterTypeException {
        left_motors_ = getRobot().getMotorFactory().createMotor("tankdrive:motors:left", "hw:tankdrive:motors:left") ;
        right_motors_ = getRobot().getMotorFactory().createMotor("tankdrive:motors:right", "hw:tankdrive:motors:right") ; 
        
        if (!left_motors_.hasPosition() || !right_motors_.hasPosition()) {
            int p1, p2 ;

            p1 = getRobot().getSettingsParser().get("hw:tankdrive:encoder:left:1").getInteger() ;
            p2 = getRobot().getSettingsParser().get("hw:tankdrive:encoder:left:2").getInteger() ;
            left_encoder_ = new Encoder(p1, p2) ;

            p1 = getRobot().getSettingsParser().get("hw:tankdrive:encoder:right:1").getInteger() ;
            p2 = getRobot().getSettingsParser().get("hw:tankdrive:encoder:right:2").getInteger() ;
            right_encoder_ = new Encoder(p1, p2) ;
        }

        if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
            left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent);
            right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent);
        }
    }
}
