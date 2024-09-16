package org.xero1425.base;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.Timer;

public class XeroTimer {
    private static int LoggerID = -1 ;
    private static final String LoggerIDName = "XeroTimer" ;
    private boolean running_ ;
    private double duration_ ;
    private double endtime_ ;
    private double start_ ;
    private String name_ ;

    public XeroTimer(String name, double duration) {
        name_ = name ;
        duration_ = duration ;
        running_ = false ;
        endtime_ = 0.0 ;        

        if (LoggerID == -1) {
            LoggerID = MessageLogger.getTheMessageLogger().registerSubsystem(LoggerIDName) ;
        }
    }

    public double getDuration() {
        return duration_ ;
    }

    public void setDuration(double dur) {
        if (running_) {
            MessageLogger logger = MessageLogger.getTheMessageLogger();
            logger.startMessage(MessageType.Error).add("Timer ").add(name_).add(" had duration updated while running - change ignored").endMessage();
        }

        duration_ = dur ;
    }

    public double elapsed() {
        return Timer.getFPGATimestamp() - start_ ;
    }

    public void start() {
        if (running_) {
            MessageLogger logger = MessageLogger.getTheMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Timer ").add(name_).add(" was started while running").endMessage();
        }

        running_ = true ;
        start_ = Timer.getFPGATimestamp() ;
        endtime_ = start_ + duration_ ;
    }

    public boolean isRunning() {
        return running_ ;
    }

    public boolean isExpired() {
        boolean ret = false ;

        if (running_ == false)
            return true ;

        if (running_ && Timer.getFPGATimestamp() > endtime_) {
            running_ = false ;
            ret = true ;
        }

        return ret ;
    }

    public String toString() {
        String ret = "XeroTimer " + name_ ;
        ret += " running " + (running_ ? "true" : "false");
        ret += " endtime " + endtime_ ;
        ret += " currenttime " + Timer.getFPGATimestamp() ;
        return ret;
    }
}