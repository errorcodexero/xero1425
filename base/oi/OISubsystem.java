package org.xero1425.base.oi;

import java.util.List;
import java.util.ArrayList;

import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This class controls the various OI devices that are used to control the robot.
/// This class is the OI subsystem for the robot.  It will add a gamepad controller to control the drive
/// base, and an OI device can be added for a game specific OI device.          
///
/// There is code in this device to manage how the robot comes up in both the practice space and on the field
/// during a competition.  In either case, when the robot code is initialized, the driver station may not be 
/// connected and therefore the OI and gamepad may not exist.  This subsystem keeps trying until the driver station
/// is connected and then initializes the OI.
///
public class OISubsystem extends Subsystem {

    /// \brief the type of gamepad to use for the tank drive
    public enum GamePadType
    {
        Xero1425Historic,           ///< The gamepad used historically by 1425
        Standard,                   ///< The gamepad used by most teams (gamepad axis raised to a power)
    }
    
    // The list of devices attached to the OI subsystem
    private List<HIDDevice> devices_ ;

    // The tankdrive subsystem
    private TankDriveSubsystem db_ ;

    // The gamepad subsystem used to control the drivebase
    private Gamepad gp_ ;

    // The gamepad index
    private int gp_index_ ;

    // If true, use
    private GamePadType gamepad_type_ ;

    // The index of the gamepad
    private final String DriverGamepadXero1425 = "xero1425_gamepad:index" ;
    private final String DriverGamepadStandard = "standard_gamepad:index" ;
    
    /// \brief Create a new OI subsystem
    /// \param parent the subsystem that manages this one
    /// \param name the name of the subsystem
    /// \param type the type of gamepad to attach
    /// \param db the drivebase to control via the gamepad
    public OISubsystem(Subsystem parent, String name, GamePadType type, TankDriveSubsystem db) {
        super(parent, name);

        devices_ = new ArrayList<HIDDevice>();
        db_ = db ;
        gp_index_ = -1 ;

        gamepad_type_ = type ;
        addTankDriveGamePad();
    }

    public GamePadType getGamePadType() {
        return gamepad_type_ ;
    }

    /// \brief return the gamepad
    /// \returns the gamepad
    public Gamepad getGamePad() {
        return gp_ ;
    }

    /// \brief returns true if this is the OI subsystem
    /// \returns true if this is the OI subsystem
    @Override
    public boolean isOI() {
        return true ;
    }

    /// \brief called when a new mode is initialized (e.g. teleop, auto, etc.).  
    /// Calls init() on each child device.
    @Override
    public void init(LoopType ltype) {
        for (HIDDevice dev : devices_)
            dev.init(ltype);
    }

    /// \brief Called each robot loop to compute the state of the OI device.  This method
    /// also tries to create a new gamepad if one does not exist.
    @Override
    public void computeMyState() throws Exception {
        // If the gp_ is null, this means an error occured when creating the gamepad at starting
        // time.  This can occur if the driver station is not connected when the robot code initializes.
        // This method keeps checking each robot loop and tries to create a gamepad until one can be created.
        if (gp_ == null) {
            addTankDriveGamePad() ;

            if (gp_ != null) {
                gp_.createStaticActions();
            }
        }

        // Iterate through each of the devices in the subsystem and if enabled, call device level computeState().
        for (HIDDevice dev : devices_) {
            if (dev.isEnabled())
                dev.computeState();
        }
    }

    /// \brief Called each robot loop to generate actions to assign for each enable OI HID device.
    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
        for(HIDDevice dev : devices_)
        {
            if (dev.isEnabled())
                dev.generateActions(seq) ;
        }
    }

    /// \brief run the subsystem
    @Override
    public void run() throws Exception {
        super.run() ;
    }

    /// \brief Called by the robot framework after all subsystems have been created.  This method
    /// calls the createStaticActions() methods on each HIDDevice managed by this OISubsystem to create
    /// any static actions.  This is necessary here to ensure the actions that need to be created have
    /// access to the subsystems on the robot.
    @Override
    public void postHWInit() throws Exception {
        //
        // This is done here because we are guarenteed to have created
        // all subsystems and established their parent child relationship
        //
        for(HIDDevice dev : devices_)
            dev.createStaticActions();
    }

    /// \brief Return the auto mode selector for the robot.  It does this by querying each managed HIDDevice
    /// and asking for the automode selector.  Note the first OI device that answers with an automode number that
    /// is not -1 is used.
    /// \returns the automode number to use
    public int getAutoModeSelector() {
        for(HIDDevice dev : devices_)
        {
            int mode = dev.getAutoModeSelector() ;
            if (mode != -1)
                return mode ;
        }

        return -1 ;
    }

    /// \brief Add a new HID device to this OI subsystem
    /// \param dev the HID device to add
    protected void addHIDDevice(HIDDevice dev) {
        devices_.add(dev) ;
    }

    private void addTankDriveGamePad() {
        if (db_ != null) {           
            MessageLogger logger = getRobot().getMessageLogger() ;

            if (gp_index_ == -1)
            {
                if (isSettingDefined(DriverGamepadXero1425))
                {
                    try {   
                        gp_index_ = getSettingsValue(DriverGamepadXero1425).getInteger() ;
                    }
                    catch(BadParameterTypeException ex) {
                        logger.startMessage(MessageType.Error) ;
                        logger.add("parameter ").addQuoted(DriverGamepadXero1425) ;
                        logger.add("exists but is not an integer type").endMessage();
                        gp_index_ = -2 ;
                    }
                    catch(MissingParameterException ex) {
                        //
                        // This will not happen, but this catch keeps the compiler happy
                        //
                    }

                    try { 
                        gp_ = new Xero1425Gamepad(this, gp_index_, db_) ;
                        addHIDDevice(gp_) ;
                        logger.startMessage(MessageType.Info) ;
                        logger.add("using historic xero1425 gamepad control").endMessage();
                    }
                    catch(Exception ex) {
                        //
                        // This is thrown if the gamepad cannot be created sucessfully, which means either
                        // the driver station has not connected to the robot, or that the joystick is not plugged in
                        //
                        gp_index_ = -1 ;
                    }
                }
                else if (isSettingDefined(DriverGamepadStandard))
                {
                    try {
                        gp_index_ = getSettingsValue(DriverGamepadStandard).getInteger() ;
                    }
                    catch(BadParameterTypeException ex) {
                        logger.startMessage(MessageType.Error) ;
                        logger.add("parameter ").addQuoted(DriverGamepadStandard) ;
                        logger.add("exists but is not an integer type").endMessage();
                        gp_index_ = -2 ;
                    }
                    catch(MissingParameterException ex) {
                        //
                        // This will not happen, but this catch keeps the compiler happy
                        //
                    }      
                    
                    try { 
                        gp_ = new StandardGamepad(this, gp_index_, db_) ;
                        addHIDDevice(gp_) ;
                        logger.startMessage(MessageType.Info) ;
                        logger.add("using standard gamepad control").endMessage();
                    }
                    catch(Exception ex) {
                        //
                        // This is thrown if the gamepad cannot be created sucessfully, which means either
                        // the driver station has not connected to the robot, or that the joystick is not plugged in
                        //
                        gp_index_ = -1 ;
                    }

                }
                else
                {
                    logger.startMessage(MessageType.Error) ;
                    logger.add("parameter ").addQuoted(DriverGamepadStandard) ;
                    logger.add("and parameter ").addQuoted(DriverGamepadXero1425) ;
                    logger.add(" do not exist").endMessage();

                    // This causes the OI to give up on trying to create the game pad
                    gp_index_ = -2 ;
                }
            }
        }
    }
}