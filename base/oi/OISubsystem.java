package org.xero1425.base.oi;

import java.util.HashMap;

import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

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
public abstract class OISubsystem extends Subsystem {
    class OIDeviceNeeded
    {
        private String name_ ;
        private boolean error_ ;
        HIDDevice dev_ ;

        public OIDeviceNeeded(String name) {
            name_ = name ;
            error_ = false ;
            dev_ = null ;
        }

        public String getName() {
            return name_ ;
        }

        public boolean hasError() {
            return error_ ;
        }

        public void setError() {
            error_ = true ;
        }

        public boolean hasDevice() {
            return dev_ != null ;
        }

        public HIDDevice getDevice() {
            return dev_ ;
        }

        void setDevice(HIDDevice dev) {
            dev_ = dev ;
        }
    }
    
    private HashMap<String, OIDeviceNeeded> devices_ ;

    public OISubsystem(Subsystem parent, String name, String [] devices) {
        super(parent, name);

        devices_ = new HashMap<String, OIDeviceNeeded>() ;
        for(String devname : devices) {
            devices_.put(devname, new OIDeviceNeeded(devname)) ;
        }
    }

    public abstract HIDDevice createDevice(String name) throws Exception ; 

    public HIDDevice getDevice(String name) {
        OIDeviceNeeded dn = devices_.get(name) ;
        if (dn == null || dn.hasError())
        {
            return null ;
        }

        return dn.getDevice() ;
    }

    /// \brief returns true if this is the OI subsystem
    /// \returns true if this is the OI subsystem
    @Override
    public boolean isOI() {
        return true ;
    }

    @Override
    public void init(LoopType ltype) {
        for(OIDeviceNeeded dev : devices_.values())
        {
            if (dev.hasDevice())
            {
                dev.getDevice().init(ltype) ;
            }
        }
    }

    @Override
    public void computeMyState() throws Exception {
        for(OIDeviceNeeded dev : devices_.values())
        {
            if (!dev.hasDevice() && !dev.hasError())
            {
                //
                // No device, but not error, so try and create the device
                //
                try {
                    HIDDevice hid = createDevice(dev.getName()) ;
                    hid.createStaticActions();
                    dev.setDevice(hid);
                }
                catch(Exception ex)
                {
                    //
                    // There was an exception in the creation process, this device will never
                    // be created
                    //
                    MessageLogger logger = getRobot().getMessageLogger() ;
                    logger.startMessage(MessageType.Error) ;
                    logger.add("Error creating OI device") ;
                    logger.add("name", dev.getName()) ;
                    logger.add("reason", ex.getMessage()) ;
                    logger.endMessage() ;
                    dev.setError();
                }
            }

            if (dev.hasDevice() && dev.getDevice().isEnabled())
                dev.getDevice().computeState(); ;
        }
    }

    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
        for(OIDeviceNeeded dev : devices_.values())
        {
            if (dev.hasError() && dev.getDevice().isEnabled())
                dev.getDevice().generateActions(seq) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    public int getAutoModeSelector() {
        for(OIDeviceNeeded dev : devices_.values())
        {
            if (dev.hasDevice() && dev.getDevice().isEnabled())
            {
                int mode = dev.getDevice().getAutoModeSelector() ;
                if (mode != -1)
                    return mode ;
            }
        }

        return -1 ;
    }
}