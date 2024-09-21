package org.xero1425.base;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.OptionalInt;
import java.util.function.Function;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.xero1425.misc.MessageDestination;
import org.xero1425.misc.MessageDestinationThumbFile;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SimArgs;
import org.xero1425.simulator.engine.SimulationEngine;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class XeroRobot extends LoggedRobot {

    public enum RobotType {
        COMPETITION_REAL,
        PRACTICE_REAL,
        COMPETITION_REPLAY,
        PRACTICE_REPLAY,
        SIMULATION
    } ;

    private static XeroRobot robot_ = null ;

    private MessageLogger logger_ ;
    private RobotPaths robot_paths_ ;
    private XeroContainer container_ ;
    private XeroAutoCommand auto_mode_ ;
    private List<XeroAutoCommand> automodes_ ;
    private SendableChooser<XeroAutoCommand> chooser_ ;
    private GenericEntry chosen_ ;
    private GenericEntry desc_ ;
    private int gamepad_port_ ;
    private int oi_port_ ;
    private AprilTagFieldLayout layout_ ;
    private boolean auto_modes_created_ ;

    private boolean connected_callback_processed_ ;
    private List<Function<Boolean,Void>> connected_callbacks_ ;

    static private int log_subsystem_cycle_count_ = 250 ;    
    private HashMap<String, Integer> periodic_count_ ;
    private HashMap<String, Integer> periodic_count_total_ ;
    private HashMap<String, Double> periodic_time_ ;
    private HashMap<String, Double> periodic_time_total_ ;
    private HashMap<String, Double> periodic_start_ ;

    private HashMap<String, ISubsystemSim> subsystems_ ;

    public XeroRobot(int gp, int oi) {
        if (robot_ != null) {
            throw new RuntimeException("XeroRobot is a singleton class") ;
        }

        gamepad_port_ = gp ;
        oi_port_ = oi ;
        auto_modes_created_ = false ;

        robot_ = this ;
        automodes_ = new ArrayList<>() ;
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());
        enableMessageLogger();
        auto_mode_ = null;
        connected_callback_processed_ = false ;
        connected_callbacks_ = new ArrayList<>() ;

        periodic_count_ = new HashMap<>() ;
        periodic_count_total_ = new HashMap<>() ;
        periodic_time_ = new HashMap<>() ;
        periodic_time_total_ = new HashMap<>() ;
        periodic_start_ = new HashMap<>() ;

        subsystems_ = new HashMap<>() ;

        // if (RobotBase.isSimulation()) {
        //     String str = SimArgs.InputFileName;
        //     if (str == null)
        //         str = getSimulationFileName() ;
                            
        //     if (str == null) {
        //         System.out.println("The code is setup to simulate, but the derived robot class did not provide a stimulus file") ;
        //         System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot") ;
        //     }
        //     else {
        //         SimulationEngine.initializeSimulator(this, logger_);
        //         addRobotSimulationModels() ;
        //         SimulationEngine.getInstance().initAll(str) ;
        //     }
        // }        
    }

    public ISubsystemSim getSubsystemByName(String name) {
        return subsystems_.get(name) ;
    }

    public abstract boolean isCharMode() ;
    public abstract boolean isTestMode() ;
    public abstract String  getSimulationFileName() ;
    public abstract String getSimulationAutoMode() ;
    protected abstract String getName() ;    
    protected abstract String getPracticeSerialNumber() ;    
    protected abstract void createCompetitionAutoModes() ;
    protected abstract void createTestAutoModes() ;
    protected abstract void addRobotSimulationModels() ;

    protected void logSubsystemCycles(int cycles) {
        log_subsystem_cycle_count_ = cycles ;
    }

    public void addConnectedCallback(Function<Boolean,Void> cb) {
        connected_callbacks_.add(cb) ;
    }
   
    public void robotInit() {
        super.robotInit() ;

        // if (RobotBase.isSimulation() && SimulationEngine.getInstance() != null)
        // {
        //     //
        //     // If we are simulating, create the simulation modules required
        //     //
        //     SimulationEngine.getInstance().createModels() ;
        // }
    }

    public AprilTagFieldLayout getFieldLayout() {
        return layout_ ;
    }

    protected void setFieldLayout(AprilTagFieldLayout layout) {
        layout_ = layout ;
    }

    protected void createAutoModes() {

        if (!DriverStation.isJoystickConnected(gamepad_port_) && !DriverStation.isJoystickConnected(oi_port_) && !XeroRobot.isSimulation()) {
            //
            // Neither the game controller nor the OI are connected.  We assume that the robot has not connected
            // yet with the driver station.  In a real event, this connection will mean we have an FMS connection
            // as the FMS connects the two.  In the school, this means the driver station computer has not connected
            // to the the robot WIFI AP yet, so we wait until we see a connection so we know what to do.
            //
            return ;
        }

        if (!auto_modes_created_) {
            if (shouldBeCompetition() || !isTestMode()) {
                createCompetitionAutoModes() ;
            }
            else {
                createTestAutoModes() ;
            }

            auto_modes_created_ = true ;
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

    public RobotType getRobotType() {
        return RobotType.COMPETITION_REAL ;
    }

    public void setContainer(XeroContainer container) {
        container_ = container ;
    }

    public XeroContainer getContainer() {
        return container_ ;
    }

    protected void addAutoMode(XeroAutoCommand mode) {
        automodes_.add(mode) ;
    }

    private void enableMessageLogger() {
        MessageDestination dest ;

        logger_ = MessageLogger.getTheMessageLogger() ;
        logger_.setTimeSource(new RobotTimeSource());

        dest = new MessageDestinationThumbFile(robot_paths_.logFileDirectory(), 250, RobotBase.isSimulation());
        logger_.addDestination(dest);
    }

    @Override
    public void robotPeriodic() {
          
        //
        // Runs the Scheduler.
        //
        double start = Timer.getFPGATimestamp() ;
        CommandScheduler.getInstance().run();   
        double delta = Timer.getFPGATimestamp() - start ;

        Logger.recordOutput("looptime", delta) ;

        if (isSimulation()) {
            SimulationEngine engine = SimulationEngine.getInstance() ;
            if (engine != null) {
                engine.run(getPeriod());
            }
        }
    }

    public void registerSubsystem(String name, ISubsystemSim subsystem) {
        subsystems_.put(name, subsystem) ;
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit() ;

        if (auto_mode_ != null) {
            auto_mode_.schedule() ;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
        createAutoModes();
    }    

    protected void enableMessages() {
    }

    public boolean isPracticeBot() {
        return RobotController.getSerialNumber().equals(getPracticeSerialNumber()) ;
    }

    public boolean isCompetitionBot() {
        return RobotBase.isReal() && !isPracticeBot() ;
    }

    public double getTime() {
        return Timer.getFPGATimestamp() ;
    }

    private void autoModeChanged(XeroAutoCommand mode) {
        chosen_.setString(mode.toString()) ;
        desc_.setString(mode.getDescription()) ;
    }

    private void autoModeChooser() {
        if (XeroRobot.isSimulation()) {
            String name = getSimulationAutoMode() ;
            for (XeroAutoCommand cmd: automodes_) {
                if (cmd.getName().equals(name)) {
                    auto_mode_ = cmd ;
                    break ;
                }
            }
        } else {
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
}
