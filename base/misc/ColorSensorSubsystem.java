package org.xero1425.base.misc;

import com.revrobotics.ColorSensorV3;

import org.xero1425.base.Subsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorSubsystem extends Subsystem {
    private I2C.Port port_ ;
    private int which_ ;
    private I2C muxdev_ ;
    private byte [] data_ = new byte[1] ;
    private byte sample_ ;
    private int count_ ;
    private ColorSensorV3 sensor_ ;
    private int[] proximity_ ;
    private Color [] colors_ ;

    public ColorSensorSubsystem(Subsystem parent, String name, I2C.Port port, int muxaddr, int count) {
        super(parent, name) ;

        port_ = port ;
        which_ = -1 ;    
        count_ = count ;

        muxdev_ = new I2C(port_, muxaddr)  ;
        select(0) ;

        sample_ = 0 ;
        for(int i = 0 ; i < count ; i++)
            sample_ |= (byte)(1 << i) ;

        sensor_ = new ColorSensorV3(port_) ;
        colors_ = new Color[count] ;
        proximity_ = new int[count] ;

        for(int i = 0 ; i < count ; i++)
            init(i) ;
    }

    public ColorSensorSubsystem(Subsystem parent, String name, I2C.Port port) {
        super(parent, name) ;

        port_ = port ;
        which_ = -1 ;
        count_ = 1 ;
        muxdev_ = null ;
        sensor_ = new ColorSensorV3(port_) ;
        colors_ = new Color[1] ;
        proximity_ = new int[1] ;

        init(0) ;
    }

    public int count() {
        return count_ ;
    }

    public Color getColor(int which) {
        return colors_[which] ;
    }

    public int getProximity(int which) {
        return proximity_[which] ;
    }

    @Override
    public void computeMyState() {
        for(int i = 0 ; i < count_ ; i++) {
            if ((sample_ & (1 << i)) != 0) {
                select(i) ;

                colors_[i] = sensor_.getColor() ;
                proximity_[i] = sensor_.getProximity() ;
            }
        }
    }

    public void enableSensor(int which) throws Exception {
        if (which >= count_) {
            throw new Exception("invalid sensor number in ColorSensorSubsystem.enableSensor(int which)") ;
        }

        sample_ |= (1 << which) ;
    }

    public void disableSensor(int which) throws Exception {
        if (which >= count_) {
            throw new Exception("invalid sensor number in ColorSensorSubsystem.enableSensor(int which)") ;
        }

        sample_ &= ~(1 << which) ;
    }

    public boolean isEnabled(int which) throws Exception {
        if (which >= count_) {
            throw new Exception("invalid sensor number in ColorSensorSubsystem.enableSensor(int which)") ;
        }

        return ((sample_ & (1 << which)) != 0) ;
    }

    // Derived class override this for different initialization
    protected ColorSensorV3.ColorSensorResolution getResolution(int which) {
        return ColorSensorV3.ColorSensorResolution.kColorSensorRes16bit ;
    }

    // Derived class override this for different initialization    
    protected ColorSensorV3.ColorSensorMeasurementRate getMeasurementRate(int which) { 
        return ColorSensorV3.ColorSensorMeasurementRate.kColorRate100ms ;
    }

    // Derived class override this for different initialization    
    protected ColorSensorV3.GainFactor getGain(int which) {
        return ColorSensorV3.GainFactor.kGain18x ;
    }

    private void init(int which) {
        select(which) ;

        ColorSensorV3.ColorSensorResolution res = getResolution(which) ;
        ColorSensorV3.ColorSensorMeasurementRate rate = getMeasurementRate(which) ;
        ColorSensorV3.GainFactor gain = getGain(which) ;
        sensor_.configureColorSensor(res, rate, gain) ;        
    }

    private void select(int which) {
        if (muxdev_ != null && which != which_) {
            data_[0] = (byte)(0x01 << which) ;
            muxdev_.writeBulk(data_);
            which_ = which ;
        }
    }
}
