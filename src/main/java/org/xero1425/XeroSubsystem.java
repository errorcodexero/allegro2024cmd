package org.xero1425;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XeroSubsystem extends SubsystemBase {
    private XeroRobot robot_ ;
    private int logger_id_ ;

    public XeroSubsystem(XeroRobot robot, String name) {
        super(name) ;

        robot_ = robot ;
        logger_id_ = getMessageLogger().registerSubsystem(name) ;
    }

    public XeroRobot getRobot() {
        return robot_ ;
    }

    protected MessageLogger getMessageLogger() {
        return robot_.getMessageLogger() ;
    }

    public int getMessageLoggerID() {
        return logger_id_ ;
    }
}
