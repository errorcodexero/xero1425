package org.xero1425.base.controllers ;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

public class TestAutoMode extends AutoMode {
    static private final String Which = "testmode:which";
    static private final String Power = "testmode:power";
    static private final String Duration = "testmode:duration";
    static private final String Distance = "testmode:distance";
    static private final String Name = "testmode:name";

    public TestAutoMode(AutoController ctrl, String name) throws BadParameterTypeException, MissingParameterException {
        super(ctrl, name) ;

        ISettingsSupplier parser = ctrl.getRobot().getSettingsParser() ;
        which_ = parser.get(Which).getInteger() ;
        power_ = parser.get(Power).getDouble() ;
        duration_ = parser.get(Duration).getDouble() ;
        position_ = parser.get(Distance).getDouble() ;
        name_ = parser.get(Name).getString() ;
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

    private int which_ ;
    private double power_ ;
    private double duration_ ;
    private double position_ ;
    private String name_ ;
}