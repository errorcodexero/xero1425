package org.xero1425.base.swervedrive;

public class SwerveStopAction extends SwerveDriveAction {

    public SwerveStopAction(SwerveDriveSubsystem subsys)  {
        super(subsys) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        for(int i = 0 ; i <= SwerveDriveSubsystem.LAST_MODULE ; i++) {
            getSubsystem().setPower(i, 0.0, 0.0);
        }

        setDone() ;
    }

    @Override
    public void run() {
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveStopAction" ;
        return ret ;
    }
}
