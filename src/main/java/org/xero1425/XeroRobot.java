package org.xero1425;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public abstract class XeroRobot extends LoggedRobot {
    private MessageLogger logger_ ;
    private RobotPaths robot_paths_ ;
    private XeroContainer container_ ;

    public XeroRobot() {
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());
        
        enableMessageLogger();
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

    protected void enableMessages() {
    }

    protected MessageLogger getMessageLogger() {
        return logger_ ;
    }

    public boolean isPracticeBot() {
        return RobotController.getSerialNumber().equals(getPracticeSerialNumber()) ;
    }

    public double getTime() {
        return RobotController.getFPGATime() ;
    }
}
