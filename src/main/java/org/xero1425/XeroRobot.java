package org.xero1425;

import org.littletonrobotics.junction.LoggedRobot;
import org.xero1425.simsupport.SimArgs;
import org.xero1425.simsupport.SimEventsManager;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public abstract class XeroRobot extends LoggedRobot {
    private MessageLogger logger_ ;
    private RobotPaths robot_paths_ ;
    private XeroContainer container_ ;
    private SimEventsManager simmgr_ ;

    public XeroRobot() {
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());
        
        enableMessageLogger();

        if (XeroRobot.isSimulation()) {
            String filename = getSimFileName() ;
            if (filename != null) {
                simmgr_ = new SimEventsManager(getMessageLogger()) ;
                simmgr_.readEventsFile(filename) ;
            }
        }
    }

    public abstract String getRobotSimFileName() ;

    public String getSimFileName() {
        if (SimArgs.InputFileName != null)
            return SimArgs.InputFileName ;

        return getRobotSimFileName() ;
    }

    public void setContainer(XeroContainer container) {
        container_ = container ;
    }

    public XeroContainer getContainer() {
        return container_ ;
    }

    public abstract String getName() ;

    // \brief return the Serial number for the practice bot roborio
    protected abstract String getPracticeSerialNumber() ;

    private void enableMessageLogger() {
        String logfile = SimArgs.LogFileName ;
        MessageDestination dest ;

        logger_ = new MessageLogger();
        logger_.setTimeSource(new RobotTimeSource());

        if (logfile != null) {
            dest = new MessageDestinationFile(logfile) ;
        }
        else {
            dest = new MessageDestinationThumbFile(robot_paths_.logFileDirectory(), 250, RobotBase.isSimulation());
        }
        logger_.addDestination(dest);   
    }

    @Override
    public void robotPeriodic() {
        if (simmgr_ != null) {
            simmgr_.processEvents(getTime()) ;
        }
    }

    protected void enableMessages() {
    }

    public MessageLogger getMessageLogger() {
        return logger_ ;
    }

    public boolean isPracticeBot() {
        return RobotController.getSerialNumber().equals(getPracticeSerialNumber()) ;
    }

    public double getTime() {
        return Timer.getFPGATimestamp() ;
    }
}
