package org.xero1425.base.controllers ;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsParser;

public class TestAutoMode extends AutoMode {
    private SettingsParser parser_ ;

    static private final String Which = "auto:testmode:which";
    static private final String Power = "auto:testmode:power";
    static private final String Duration = "auto:testmode:duration";
    static private final String Distance = "auto:testmode:distance";
    static private final String Name = "auto:testmode:name";
    static private final String Prefix = "auto:testmode:" ;

    public TestAutoMode(AutoController ctrl, String name) throws BadParameterTypeException, MissingParameterException {
        super(ctrl, name) ;

        parser_ = ctrl.getRobot().getSettingsParser() ;
        which_ = parser_.get(Which).getInteger() ;
        power_ = parser_.get(Power).getDouble() ;
        duration_ = parser_.get(Duration).getDouble() ;
        position_ = parser_.get(Distance).getDouble() ;
        name_ = parser_.get(Name).getString() ;
    }

    protected int getTestNumber() {
        return which_;
    }

    protected double getPower() {
        return power_ ;
    }

    protected double getDuration() {
        return duration_ ;
    }

    protected double getPosition() {
        return position_ ;
    }

    protected String getNameParam() {
        return name_ ;
    }

    protected double getNamedDouble(String name) throws BadParameterTypeException, MissingParameterException {
        return parser_.get(Prefix + name).getDouble() ;
    }

    private int which_ ;
    private double power_ ;
    private double duration_ ;
    private double position_ ;
    private String name_ ;
}