package org.xero1425.base;

public class XeroContainer {
    private XeroRobot robot_ ;


    protected XeroContainer(XeroRobot robot) {
        robot_ = robot ;
        robot_.setContainer(this) ;
    }

    protected XeroRobot getRobot() {
        return robot_ ;
    }
}
