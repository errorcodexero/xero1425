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

/// \brief This class controls the various OI devices that are used to control the robot.
/// This class is the OI subsystem for the robot.  It will add a gamepad controller to control the drive
/// base, and an OI device can be added for a game specific OI device.
///
/// This class has requirements for the settings file.  The following entries must be in the settings file or
/// this class will not work properly
///
///      # The driver station index of the driver gamepad
///      hw:driverstation:hid:driver                                             0     
///
public class OISubsystem extends Subsystem {
    
    private List<HIDDevice> devices_ ;
    private TankDriveGamepad gp_ ;
    
    private final static String DriverGamepadHIDIndexName = "hw:driverstation:hid:driver";

    public OISubsystem(Subsystem parent, String name, TankDriveSubsystem db) {
        super(parent, name);

        devices_ = new ArrayList<HIDDevice>();

        if (db != null) {           
            MessageLogger logger = getRobot().getMessageLogger() ;
            int index;

            try {
                index = getRobot().getSettingsParser().get(DriverGamepadHIDIndexName).getInteger();
            } catch (BadParameterTypeException e) {
                logger.startMessage(MessageType.Error) ;
                logger.add("parameter ").addQuoted(DriverGamepadHIDIndexName) ;
                logger.add(" exists, but is not an integer").endMessage();
                index = -1 ;
            } catch (MissingParameterException e) {
                logger.startMessage(MessageType.Error) ;
                logger.add("parameter ").addQuoted(DriverGamepadHIDIndexName) ;
                logger.add(" does not exist").endMessage();
                index = -1 ;            
            }
            
            if (index != -1) {
                try {
                    gp_ = new TankDriveGamepad(this, index, db) ;
                    addHIDDevice(gp_);
                }
                catch(Exception ex) {
                    logger.startMessage(MessageType.Error) ;
                    logger.add("driver gamepad HID device was not created ").endMessage();
                }
            }
        }
    }

    /// \brief returns true if this is the OI subsystem
    /// \returns true if this is the OI subsystem
    @Override
    public boolean isOI() {
        return true ;
    }

    @Override
    public void init(LoopType ltype) {
        for (HIDDevice dev : devices_)
            dev.init(ltype);
    }

    @Override
    public void computeMyState() throws Exception {
        for (HIDDevice dev : devices_) {
            if (dev.isEnabled())
                dev.computeState();
        }
    }

    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
        for(HIDDevice dev : devices_)
            dev.generateActions(seq) ;
    }

    @Override
    public void run() {
    }

    @Override
    public void postHWInit() throws Exception {
        //
        // This is done here because we are guarenteed to have created
        // all subsystems and established their parent child relationship
        //
        for(HIDDevice dev : devices_)
            dev.createStaticActions();
    }

    public int getAutoModeSelector() {
        for(HIDDevice dev : devices_)
        {
            int mode = dev.getAutoModeSelector() ;
            if (mode != -1)
                return mode ;
        }

        return -1 ;
    }

    protected void addHIDDevice(HIDDevice dev) {
        devices_.add(dev) ;
    }
} ;