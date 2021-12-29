package org.xero1425.base.controllers ;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

/// \brief This class is the base class for the standard test mode auto mode that is
/// created for each robot.  There is expected to be a derived class that is robot specific
/// that supplies the specific test modes needed.
public class TestAutoMode extends AutoMode {
    
    // The test mode to run
    private int which_ ;

    // The power number from the settings file (see settings names below)
    private double power_ ;

    // The duration number from the settings file (see settings names below)
    private double duration_ ;

    // The position number from the settings file (see settings names below)    
    private double position_ ;

    // The name string from the settings file (see settings names below)    
    private String name_ ;

    // The settings name string for the which value
    static private final String Which = "testmode:which";

    // The settings name string for the power value    
    static private final String Power = "testmode:power";

    // The settings name string for the duration value    
    static private final String Duration = "testmode:duration";

    // The settings name string for the distance value    
    static private final String Distance = "testmode:distance";

    // The settings name string for the name value    
    static private final String Name = "testmode:name";

    /// \brief Create a new test automode
    /// \param ctrl the automode controller that manages this mode
    /// \param name the name of the automodes
    public TestAutoMode(AutoController ctrl, String name) throws BadParameterTypeException, MissingParameterException {
        super(ctrl, name) ;

        ISettingsSupplier parser = ctrl.getRobot().getSettingsSupplier() ;
        which_ = parser.get(Which).getInteger() ;
        power_ = parser.get(Power).getDouble() ;
        duration_ = parser.get(Duration).getDouble() ;
        position_ = parser.get(Distance).getDouble() ;
        name_ = parser.get(Name).getString() ;
    }

    /// \brief Returns the number of the test to run
    /// \returns the number of the test to run.
    protected int getTestNumber() {
        return which_;
    }

    /// \brief Returns the power value from the settings file
    /// \returns the power value from the settings file
    protected double getPower() {
        return power_ ;
    }

    /// \brief Returns the duration value from the settings file
    /// \returns the duration value from the settings file    
    protected double getDuration() {
        return duration_ ;
    }

    /// \brief Returns the position value from the settings file
    /// \returns the position value from the settings file    
    protected double getPosition() {
        return position_ ;
    }

    /// \brief Returns the name value from the settings file
    /// \returns the name value from the settings file    
    protected String getNameParam() {
        return name_ ;
    }
}