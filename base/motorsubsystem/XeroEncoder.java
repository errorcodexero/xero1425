package org.xero1425.base.motorsubsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

public class XeroEncoder {
    
    private Encoder quad_ ;
    private double quad_m_ ;
    private double quad_b_ ;
    private MotorController motor_ ;
    private AnalogInput analog_ ;
    private Counter pwm_ ;
    private EncoderMapper mapper_ ;

    public XeroEncoder(XeroRobot robot, String cname, boolean angular, MotorController ctrl)
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
            BadMotorRequestException {
        createEncoder(robot, cname, ctrl);
    }

    private void createEncoder(XeroRobot robot, String cname, MotorController ctrl)
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
            BadMotorRequestException {

        ISettingsSupplier settings = robot.getSettingsParser() ;                
        String type = settings.get(cname + ":type").getString() ;

        if (type.equals("motor")) {
            createMotorEncoder(robot, cname, ctrl) ;
        }
        else if (type.equals("quad")) {
            createQuadEncoder(robot, cname, ctrl);
        }
        else if (type.equals("analog")) {
            createAnalogEncoder(robot, cname);
        }
        else if (type.equals("pwm")) {
            createPWMEncoder(robot, cname);
        }
        else {
            throw new EncoderConfigException("motor '" + cname + " - unknown encoder type '" + type + "' - expected 'motor' or 'analog' or 'quad' or 'pwm'") ;
        }

        if (pwm_ == null && analog_ == null && quad_ == null && motor_ == null)
            throw new EncoderConfigException("motor '" + cname + "' - must define a QUAD, PWM, MOTOR, or ANALOG encoder");
    }

    public double getRawCount() {
        double result = 0.0 ;

        try {
            if (motor_ != null)
                result = motor_.getPosition() ;
            else if (quad_ != null)
                result = quad_.get() ;
            else if (analog_ != null)
                result = analog_.getVoltage() ;
            else if (pwm_ != null)
                result = pwm_.getPeriod() ;
        }
        catch(Exception ex) {
            result = 0.0 ;
        }

        return result ;
    }

    public double getPosition() {
        double result = 0.0;

        try {
            if (motor_ != null) {
                result = motor_.getPosition() * quad_m_ + quad_b_;
            }
            else if (quad_ != null) {
                result = quad_.get() * quad_m_ + quad_b_ ;
            }
            else if (analog_ != null) {
                result = mapper_.toRobot(analog_.getVoltage()) ;
            }
            else if (pwm_ != null) {
                result = mapper_.toRobot(pwm_.getPeriod()) ;
            }
        } 
        catch (Exception ex) 
        {
            //
            // THis should never happen, but in case it does
            //
            result = 0.0 ;
        }

        return result;
    }

    private double getAbsolutePosition() {
        double result = 0.0;

        if (analog_ != null) {
            result = mapper_.toRobot(analog_.getVoltage()) ;
        }
        else if (pwm_ != null) {
            result = mapper_.toRobot(pwm_.getPeriod()) ;
        }
        return result;        
    }

    public void reset() {
        try {
            if (motor_ != null)
                motor_.resetEncoder();
            else if (quad_ != null)
                quad_.reset() ;

            calibrate() ;
        }
        catch(Exception ex) {

        }
    }

    public void calibrate(double pos) {
        if (quad_ != null)
            quad_.reset() ;
        else if (motor_ != null)
        {
            try {
                motor_.resetEncoder();
            }
            catch(Exception ex) {                
            }
        }

        quad_b_ = pos ;
    }

    public void calibrate() {
        if ((quad_ != null || motor_ != null) && (analog_ != null && pwm_ != null))
        {
            //
            // We have one of QUAD or MOTOR encoder which are relative
            // We have one of ANALOG or PWM encoder which are absolute
            //
            // Use the absolute encoder to calibrate the relative encoder
            //
            calibrate(getAbsolutePosition());
        }
    }

    private void createMotorEncoder(XeroRobot robot, String cname, MotorController ctrl)
                throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
                BadMotorRequestException {

        ISettingsSupplier settings = robot.getSettingsParser() ;                    

        motor_ = ctrl ;
        if (!motor_.hasPosition())
            throw new EncoderConfigException("motor '" + cname + "' - motor does not have internal encoder");
    
        quad_m_ = settings.get(cname + ":m").getDouble() ;
        quad_b_ = settings.get(cname + ":b").getDouble() ;
    }

    private void createQuadEncoder(XeroRobot robot, String cname, MotorController ctrl)
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
            BadMotorRequestException {

        ISettingsSupplier settings = robot.getSettingsParser() ;

        int i1 = settings.get(cname + ":dinput1").getInteger() ;
        int i2 = settings.get(cname + ":dinput2").getInteger() ;        

        quad_ = new Encoder(i1, i2) ;
        quad_m_ = settings.get(cname + ":m").getDouble() ;
        quad_b_ = settings.get(cname + ":b").getDouble() ;
    }

    private void createAnalogEncoder(XeroRobot robot, String cname)
            throws BadParameterTypeException, MissingParameterException {
        ISettingsSupplier settings = robot.getSettingsParser() ;

        int a = settings.get(cname+":ainput").getInteger() ;
        analog_ = new AnalogInput(a) ;

        double rmin, rmax ;
        double emin, emax ;
        double rc, ec ;

        rmin = settings.get(cname + ":rmin").getDouble() ;
        rmax = settings.get(cname + ":rmax").getDouble() ;
        emin = settings.get(cname + ":emin").getDouble() ;
        emax = settings.get(cname + ":emax").getDouble() ;
        rc = settings.get(cname + ":rc").getDouble() ;
        ec = settings.get(cname + ":ec").getDouble() ;

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec) ;
    }

    private void createPWMEncoder(XeroRobot robot, String cname)
            throws BadParameterTypeException, MissingParameterException {
                
        ISettingsSupplier settings = robot.getSettingsParser() ;
                
        int a = settings.get(cname + ":dinput").getInteger() ;
        pwm_ = new Counter(a) ;
        pwm_.setSemiPeriodMode(true);

        double rmin, rmax ;
        double emin, emax ;
        double rc, ec ;

        rmin = settings.get(cname + ":rmin").getDouble() ;
        rmax = settings.get(cname + ":rmax").getDouble() ;
        emin = settings.get(cname + ":emin").getDouble() ;
        emax = settings.get(cname + ":emax").getDouble() ;
        rc = settings.get(cname + ":rc").getDouble() ;
        ec = settings.get(cname + ":ec").getDouble() ;

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec) ;
    }

} ;