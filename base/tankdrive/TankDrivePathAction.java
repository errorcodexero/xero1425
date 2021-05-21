package org.xero1425.base.tankdrive;

import org.xero1425.misc.XeroPath;

public abstract class TankDrivePathAction extends TankDriveAction {
    private String path_name_ ;
    private XeroPath path_ ;

    public TankDrivePathAction(TankDriveSubsystem sub, String pathname) {
        super(sub) ;

        path_name_ = pathname ;
    }

    public String getPathName() {
        return path_name_ ;
    }

    public XeroPath getPath() {
        return path_ ;
    }

    public void start() throws Exception {
        path_ = getSubsystem().getRobot().getPathManager().getPath(getPathName()) ;
    }
}
