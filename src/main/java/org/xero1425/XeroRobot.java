package org.xero1425;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;

import org.littletonrobotics.junction.LoggedRobot;
import org.xero1425.simsupport.SimArgs;
import org.xero1425.simsupport.SimEventsManager;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public abstract class XeroRobot extends LoggedRobot {

    private static XeroRobot robot_ = null ;

    private MessageLogger logger_ ;
    private RobotPaths robot_paths_ ;
    private XeroContainer container_ ;
    private SimEventsManager simmgr_ ;
    private XeroAutoCommand auto_mode_ ;
    private List<XeroAutoCommand> automodes_ ;
    private SendableChooser<XeroAutoCommand> chooser_ ;
    private GenericEntry chosen_ ;
    private GenericEntry desc_ ;
    private int gamepad_port_ ;
    private int oi_port_ ;
    private AprilTagFieldLayout layout_ ;

    public XeroRobot(int gp, int oi) {
        if (robot_ != null) {
            throw new RuntimeException("XeroRobot is a singleton class") ;
        }

        gamepad_port_ = gp ;
        oi_port_ = oi ;

        robot_ = this ;
        automodes_ = new ArrayList<>() ;        
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());       
        enableMessageLogger();
        auto_mode_ = null; 
    }

    public AprilTagFieldLayout getFieldLayout() {
        return layout_ ;
    }

    protected void setFieldLayout(AprilTagFieldLayout layout) {
        layout_ = layout ;
    }

    protected abstract void createCompetitionAutoModes() ;
    protected abstract void createTestAutoModes() ;
    protected abstract boolean needTestModes() ;

    protected void createAutoModes() {

        if (!DriverStation.isJoystickConnected(gamepad_port_) && !DriverStation.isJoystickConnected(oi_port_)) {
            //
            // Neither the game controller nor the OI are connected.  We assume that the robot has not connected
            // yet with the driver station.  In a real event, this connection will mean we have an FMS connection
            // as the FMS connects the two.  In the school, this means the driver station computer has not connected
            // to the the rboto WIFI AP yet, so we wait until we see a connection so we know what to do.
            //
            return ;
        }        

        if (shouldBeCompetition() || !needTestModes()) {
            createCompetitionAutoModes() ;
        }
        else {
            createTestAutoModes() ;
        }

        autoModeChooser();        
    }

    protected boolean shouldBeCompetition() {
        if (DriverStation.isFMSAttached())
            return true ;

        OptionalInt loc = DriverStation.getLocation() ;
        if (loc.isPresent() && loc.getAsInt() == 3)
            return true ;

        return false ;
    }

    public abstract boolean isCharacterizing() ;

    public static boolean isChar() {
        return robot_.isCharacterizing() ;
    }

    public static boolean isPractice() {
        return RobotBase.isReal() && robot_.isPracticeBot() ;
    }

    public static boolean isCompetition() {
        return RobotBase.isReal() && !robot_.isPracticeBot() ;
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


    protected void addAutoMode(XeroAutoCommand mode) {
        automodes_.add(mode) ;
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
    public void disabledPeriodic() {
    }

    @Override
    public void robotPeriodic() {
        if (simmgr_ != null) {
            simmgr_.processEvents(getTime()) ;
        }

        createAutoModes();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit() ;

        if (auto_mode_ != null) {
            auto_mode_.schedule() ;
        }
    }

    @Override
    public void teleopInit() {
        if (auto_mode_ != null) {
            auto_mode_.cancel() ;
        }
    }

    @Override
    public void simulationInit() {
        String filename = getSimFileName() ;
        if (filename != null) {
            simmgr_ = new SimEventsManager(getMessageLogger()) ;
            simmgr_.readEventsFile(filename) ;
            simmgr_.initialize() ;            
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

    private void autoModeChanged(XeroAutoCommand mode) {
        chosen_.setString(mode.toString()) ;
        desc_.setString(mode.getDescription()) ;
    }

    private void autoModeChooser() {
        if (chooser_ == null && automodes_.size() > 0) {
            chooser_ = new SendableChooser<>();
            chooser_.onChange((mode) -> autoModeChanged(mode)) ;
            boolean defaultSet = false ;
            for (XeroAutoCommand mode : automodes_) {
                chooser_.addOption(mode.toString(), mode) ;
                if (!defaultSet) {
                    auto_mode_ = mode ;
                    chooser_.setDefaultOption(mode.toString(), mode) ;
                    defaultSet = true ;
                }
            }

            ShuffleboardTab tab = Shuffleboard.getTab("AutoMode") ;
            chosen_ = tab.add("Auto Mode Selected", auto_mode_.toString()).withSize(2, 1).withPosition(3, 0).getEntry() ;
            desc_ = tab.add("Auto Mode Description", auto_mode_.toString()).withSize(5, 2).withPosition(0, 1).getEntry() ;
            tab.add("Auto Mode Selecter", chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0) ;
        }

        if (chooser_ != null) {
            auto_mode_ = chooser_.getSelected() ;
        }
    }
}
