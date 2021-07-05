package org.xero1425.simulator.models;

public class SwerveDriveModuleModel {
    private SimMotorController steer_ ;
    private SimMotorController drive_ ;

    public SwerveDriveModuleModel(SwerveDriveModel model, String name) throws Exception {
        String motorname = name + "steer" ;
        steer_ = new SimMotorController(model, motorname) ;
        if (!steer_.createMotor())
        {
            throw new Exception("cannot create motor '" + motorname) ;
        }

        motorname = name + "drive" ;
        drive_ = new SimMotorController(model, motorname) ;
        if (!drive_.createMotor())
        {
            throw new Exception("cannot create motor '" + motorname) ;
        }
    }

    public void run(double dt) {
    }

    public double getSteerPower() {
        return steer_.getPower() ;
    }

    public double getDrivePower() {
        return drive_.getPower() ;
    }

}
