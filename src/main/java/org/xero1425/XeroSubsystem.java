package org.xero1425;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class XeroSubsystem extends SubsystemBase {
    private XeroRobot robot_ ;
    private int logger_id_ ;
    private int periodic_count_ ;
    private double periodic_time_ ;
    private double periodic_start_ ;

    public XeroSubsystem(XeroRobot robot, String name) {
        super(name) ;

        robot_ = robot ;
        logger_id_ = getMessageLogger().registerSubsystem(name) ;

        periodic_count_ = 0 ;
        periodic_time_ = 0.0 ;
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
        periodic_start_ = Timer.getFPGATimestamp() * 1000.0 ;
    }

    protected void periodicEnd() {
        MessageLogger logger = MessageLogger.getTheMessageLogger() ;

        periodic_count_++ ;
        
        double now = Timer.getFPGATimestamp() * 1000.0 ;
        double elapsed = now - periodic_start_ ;
        periodic_time_ += elapsed ;

        // logger.startMessage(MessageType.Info) ;
        // logger.add("start", periodic_start_, "%.15f") ;
        // logger.add("now", now, "%.15f") ;
        // logger.add("Elapsed", elapsed, "%.15f") ;
        // logger.endMessage();

        if ((periodic_count_ % 250) == 0) {
            double avg = periodic_time_ / periodic_count_  ;

            logger.startMessage(MessageType.Info) ;
            logger.add("Name", getName()) ;
            logger.add("count", periodic_count_) ;
            logger.add("average", avg, "%.6f") ;
            logger.endMessage();
        }
    }
}
