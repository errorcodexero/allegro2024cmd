package org.xero1425;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class XeroSubsystem extends SubsystemBase {
    static private boolean log_subsystem_times_ = false ;
    static private int log_subsystem_cycle_count_ = 250 ;
    private XeroRobot robot_ ;
    private int logger_id_ ;
    private int periodic_count_ ;
    private int periodic_count_total_ ;
    private double periodic_time_ ;
    private double periodic_time_total_ ;
    private double periodic_start_ ;

    public XeroSubsystem(XeroRobot robot, String name) {
        super(name) ;

        robot_ = robot ;
        logger_id_ = getMessageLogger().registerSubsystem(name) ;

        periodic_count_ = 0 ;
        periodic_time_ = 0.0 ;
        periodic_count_total_ = 0 ;
        periodic_time_total_ = 0.0 ;
    }

    public static void setLogSubsystemTimes(boolean log) {
        log_subsystem_times_ = log ;
    }

    public static void setLogSubsystemCycleCount(int count) {
        log_subsystem_cycle_count_ = count ;
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
        periodic_count_total_++ ;
        
        double elapsed = Timer.getFPGATimestamp() * 1000.0 - periodic_start_ ;
        periodic_time_ += elapsed ;
        periodic_time_total_ += elapsed ;

        if (log_subsystem_times_ && periodic_count_ == log_subsystem_cycle_count_) {
            double curavg = periodic_time_ / periodic_count_  ;
            double totavg = periodic_time_total_ / periodic_count_total_ ;

            logger.startMessage(MessageType.Info) ;
            logger.add("Name", getName()) ;
            logger.add("currant", curavg, "%.6f") ;
            logger.add("total", totavg, "%.6f") ;
            logger.endMessage();

            periodic_count_ = 0 ;
            periodic_time_ = 0.0 ;
        }
    }
}
