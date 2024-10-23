package org.xero1425.base;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.xero1425.misc.MessageDestination;
import org.xero1425.misc.MessageDestinationThumbFile;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SimArgs;
import org.xero1425.paths.XeroPathManager;
import org.xero1425.paths.XeroPathType;
import org.xero1425.simulator.engine.SimulationEngine;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    // The path following paths
    private XeroPathManager paths_ ;    

    private MessageLogger logger_ ;
    private RobotPaths robot_paths_ ;
    private XeroContainer container_ ;
    private XeroAutoCommand auto_mode_ ;
    private List<XeroAutoCommand> automodes_ ;
    private SendableChooser<XeroAutoCommand> chooser_ ;
    private GenericEntry chosen_ ;
    private GenericEntry desc_ ;
    // private int gamepad_port_ ;
    // private int oi_port_ ;
    private AprilTagFieldLayout layout_ ;
    private boolean chooser_populated_ ;

    private XboxController gamepad_ ;
    private double rumble_ ;
    private double stop_rumble_time_ ;

    private Alliance alliance_ ;

    private HashMap<String, Double> periodic_times_ ;

    private HashMap<String, ISubsystemSim> subsystems_ ;

    public XeroRobot(int gp, int oi) {
        if (robot_ != null) {
            throw new RuntimeException("XeroRobot is a singleton class") ;
        }

        periodic_times_ = new HashMap<>() ;

        // gamepad_port_ = gp ;
        // oi_port_ = oi ;
        alliance_ = Alliance.Red ;

        robot_ = this ;
        automodes_ = new ArrayList<>() ;
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());
        enableMessageLogger();
        auto_mode_ = null;

        subsystems_ = new HashMap<>() ;

        if (RobotBase.isSimulation()) {
            String str = SimArgs.InputFileName;
            if (str == null)
                str = getSimulationFileName() ;
                            
            if (str == null) {
                System.out.println("The code is setup to simulate, but the derived robot class did not provide a stimulus file") ;
                System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot") ;
            }
            else {
                SimulationEngine.initializeSimulator(this, logger_);
                addRobotSimulationModels() ;
                SimulationEngine.getInstance().initAll(str) ;
            }
        }    
        
        paths_ = new XeroPathManager(robot_paths_.pathsDirectory(), XeroPathType.SwerveHolonomic) ;
        try {
            loadPathsFile();
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("caught exception reading path files -").add(ex.getMessage()).endMessage();
        }
    }

    public void startPeriodic(String name) {
        periodic_times_.put(name, Timer.getFPGATimestamp()) ;
    }

    public void endPeriodic(String name) {
        double runtime = Double.NaN;
        if (periodic_times_.containsKey(name)) {
            runtime = Timer.getFPGATimestamp() - periodic_times_.get(name) ;
        }

        Logger.recordOutput("timingsMS:" + name, 1000 * runtime) ;
    }

    public ISubsystemSim getSubsystemByName(String name) {
        return subsystems_.get(name) ;
    }

    public void setDriveController(XboxController ctrl) {
        gamepad_ = ctrl ;
    }

    public void setRumble(double value, double duration) {
        rumble_ = value ;
        stop_rumble_time_ = Timer.getFPGATimestamp() + duration ;
        gamepad_.setRumble(RumbleType.kBothRumble, value);
    }

    public double getRumble() {
        return rumble_ ;
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

    public XeroPathManager getPathManager() {
        return paths_ ;
    }

    public void robotInit() {
        super.robotInit() ;

        if (RobotBase.isSimulation() && SimulationEngine.getInstance() != null)
        {
            //
            // If we are simulating, create the simulation modules required
            //
            SimulationEngine.getInstance().createModels() ;
        }
    }

    public AprilTagFieldLayout getFieldLayout() {
        return layout_ ;
    }

    protected void setFieldLayout(AprilTagFieldLayout layout) {
        layout_ = layout ;
    }

    private void updateAutoModeAlliance(Alliance a) {
        for(XeroAutoCommand mode : automodes_) {
            mode.setAlliance(a);
        }
    }

    protected void createAutoModes() {
        //
        // Create auto modes, assume red alliance
        //
        createCompetitionAutoModes();
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

    /// \brief load the paths file from the paths file directory
    protected void loadPathsFile() throws Exception {
        try (Stream<Path> walk = Files.walk(Paths.get(paths_.getBaseDir()))) {
            List<String> result = walk.map(x -> x.toString()).filter(f -> f.endsWith("-main.csv")).collect(Collectors.toList());
            for(String name : result) {
                int index = name.lastIndexOf(File.separator) ;
                if (index != -1) {
                    name = name.substring(index + 1) ;
                    name = name.substring(0, name.length() - 9) ;
                    paths_.loadPath(name) ;
                }
            }
        }
        catch(IOException ex) {
        }
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
        CommandScheduler.getInstance().run();   

        if (isSimulation()) {
            SimulationEngine engine = SimulationEngine.getInstance() ;
            if (engine != null) {
                engine.run(getPeriod());
            }
        }

        if (gamepad_ != null && rumble_ != 0.0) {
            if (Timer.getFPGATimestamp() > stop_rumble_time_) {
                rumble_ = 0.0 ;
                gamepad_.setRumble(RumbleType.kBothRumble, 0.0);
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
            Logger.recordMetadata("auto-mode-run", auto_mode_.getName()) ;
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
        //
        // Creates and empty automode chooser
        //
        if (chooser_ == null) {
            createAutoModeChooser();
        }

        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            //
            // Create the auto modes
            //
            if (automodes_.size() == 0) {
                //
                // This assumes red auto modes
                //
                createAutoModes() ;
                updateAutoModeAlliance(alliance_) ;
            }
            
            //
            // If the value alliance_ defaults to red.  If the alliance
            // has changed, update the auto modes to reflect the new alliance
            // color.
            //
            if (alliance_ != alliance.get()) {
                updateAutoModeAlliance(alliance.get());
            }

            alliance_ = alliance.get() ;

            //
            // Populate the auto mode chooser if necessary
            //
            if (automodes_.size() > 0 && !chooser_populated_) { 
                populateAutoModeChooser();
                chooser_populated_ = true ;
            }
        }
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

    private void populateAutoModeChooser() {
        boolean defaultSet = false ;
        for (XeroAutoCommand mode : automodes_) {
            chooser_.addOption(mode.toString(), mode) ;
            if (!defaultSet) {
                auto_mode_ = mode ;
                chooser_.setDefaultOption(mode.toString(), mode) ;
                defaultSet = true ;

                autoModeChanged(mode) ;
            }
        }
    }

    private void createAutoModeChooser() {
        if (chooser_ == null) {
            chooser_ = new SendableChooser<>();
            chooser_.onChange((mode) -> autoModeChanged(mode)) ;

            ShuffleboardTab tab = Shuffleboard.getTab("AutoMode") ;
            chosen_ = tab.add("Auto Mode Selected", "").withSize(2, 1).withPosition(3, 0).getEntry() ;
            desc_ = tab.add("Auto Mode Description", "").withSize(5, 2).withPosition(0, 1).getEntry() ;
            tab.add("Auto Mode Selecter", chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0) ;
        }
    }
}
