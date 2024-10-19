package org.xero1425.base;

import java.util.Map;
import org.xero1425.misc.MessageLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class XeroSubsystem extends SubsystemBase implements ISubsystemSim {

    private XeroRobot robot_ ;
    private int logger_id_ ;
    private boolean verbose_ = true ;

    public XeroSubsystem(XeroRobot robot, String name) {
        super(name) ;

        robot_ = robot ;
        logger_id_ = MessageLogger.getTheMessageLogger().registerSubsystem(name) ;

        robot_.registerSubsystem(name, this) ;
    }

    public void startPeriodic() {
        getRobot().startPeriodic(getName()) ;
    }

    public void endPeriodic() {
        getRobot().endPeriodic(getName()) ;
    }

    public void setVerbose(boolean value) {
        verbose_ = value ;
    }

    public boolean getVerbose() {
        return verbose_ || XeroRobot.isSimulation() ;
    }

    public XeroRobot getRobot() {
        return robot_ ;
    }

    public int getMessageLoggerID() {
        return logger_id_ ;
    }

    public abstract Map<String, TalonFX> getCTREMotors() ;
}
