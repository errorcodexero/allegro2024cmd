package org.xero1425;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.xero1425.simsupport.SimArgs;
import org.xero1425.simsupport.SimEventsManager;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public abstract class XeroRobot extends LoggedRobot {
    private MessageLogger logger_ ;
    private RobotPaths robot_paths_ ;
    private XeroContainer container_ ;
    private SimEventsManager simmgr_ ;
    private XeroAutoMode auto_mode_ ;
    private List<XeroAutoMode> automodes_ ;
    private SendableChooser<XeroAutoMode> chooser_ ;

    public XeroRobot() {
        automodes_ = new ArrayList<>() ;        
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());       
        enableMessageLogger();
        auto_mode_ = null; 
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


    protected void addAutoMode(XeroAutoMode mode) {
        automodes_.add(mode) ;
    }

    List<XeroAutoMode> getAutoModes() {
        return automodes_ ;
    }    

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

        autoModeChooser();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit() ;

        if (auto_mode_ != null) {
            auto_mode_.getCommand().schedule() ;
        }
    }

    @Override
    public void teleopInit() {
        if (auto_mode_ != null) {
            auto_mode_.getCommand().cancel() ;
        }
    }

    @Override
    public void simulationInit() {
        String filename = getSimFileName() ;
        if (filename != null) {
            simmgr_ = new SimEventsManager(getMessageLogger()) ;
            simmgr_.readEventsFile(filename) ;
        }

        simmgr_.initialize() ;
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

    void autoModeChooser() {
        if (chooser_ == null && automodes_.size() > 0) {
            chooser_ = new SendableChooser<>() ;
            boolean defaultSet = false ;
            for (XeroAutoMode mode : automodes_) {
                chooser_.addOption(mode.toString(), mode) ;
                if (!defaultSet) {
                    chooser_.setDefaultOption(mode.toString(), mode) ;
                    defaultSet = true ;
                }
            }

            Shuffleboard.getTab("AutoMode").add("AutoMode", chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser) ;            
        }

        if (chooser_ != null) {
            auto_mode_ = chooser_.getSelected() ;
        }
    }
}
