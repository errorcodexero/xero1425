package org.xero1425.base.controllers;

import org.xero1425.base.actions.SequenceAction;

public class AutoMode extends SequenceAction {
    public AutoMode(AutoController ctrl, String name) {
        super(ctrl.getRobot().getMessageLogger()); 

        ctrl_ = ctrl ;
        name_ = name ;

        ctrl.addAutoMode(this) ;
    }

    public String getName() {
        return name_ ;
    }

    public void update(String gdata) throws Exception {
    }

    protected AutoController getAutoController() {
        return ctrl_ ;
    }

    private String name_ ;
    private AutoController ctrl_ ;
}