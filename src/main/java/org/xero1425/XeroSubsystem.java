package org.xero1425;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class XeroSubsystem extends SubsystemBase {

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

    public abstract List<TalonFX> getCTREMotors() ;
    public abstract List<CANSparkBase> getRevRoboticsMotors() ;

    protected void periodicStart() {        
        robot_.periodicStart(getName()) ;
    }

    protected void periodicEnd() {
        robot_.periodicEnd(getName());
    }
}
