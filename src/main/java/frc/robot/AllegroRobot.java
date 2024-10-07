// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SimArgs;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.automodes.competition.DriveStraight;
import frc.robot.automodes.competition.FourNoteDynamicCommand;
import frc.robot.automodes.competition.FourNoteQuickCommand;
import frc.robot.automodes.competition.JustShootCommand;
import frc.robot.automodes.competition.ThreeNotePathsCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.oi.OIConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class AllegroRobot extends XeroRobot {
    private static final boolean kLogToNetworkTables = false ;

    private AllegroContainer container_;

    public AllegroRobot() {
        super(OIConstants.kDriverControllerPort, OIConstants.kOIControllerPort) ;
        setFieldLayout(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()) ;
    }  

    protected void addRobotSimulationModels() {
        ModelFactory factory = SimulationEngine.getInstance().getModelFactory();
        factory.registerModel("intake-shooter", "frc.models.IntakeShooterModel");
        factory.registerModel("amp-trap", "frc.models.AmpTrapModel");
        factory.registerModel("allegro-oi", "frc.models.AllegroOIModel");    
    }      

    @Override
    public String getSimulationFileName() {
        String ret = SimArgs.InputFileName;
        if (ret != null)
            return ret;

        return "collectshootxfer";
    }

    @Override
    public String getSimulationAutoMode() {
        return "three-note-paths" ;
    }

    @Override
    public boolean isCharMode() {
        return RobotConstants.kCharMode;
    }

    public boolean isTestMode() {
        return RobotConstants.kTestMode ;
    }

    @Override
    public String getName() {
        return "allegro";
    }

    @Override
    public String getPracticeSerialNumber() {
        return "032414C9" ;
    }

    @Override
    public void createCompetitionAutoModes() {
        if (container_ != null && container_.getDriveTrain() != null) {
            addAutoMode(new FourNoteDynamicCommand(this, container_));
            addAutoMode(new FourNoteQuickCommand(this, container_, true)) ;
            addAutoMode(new FourNoteQuickCommand(this, container_, false)) ;            
            addAutoMode(new ThreeNotePathsCommand(this, container_)) ;
            addAutoMode(new JustShootCommand(this, container_)) ;
            addAutoMode(new DriveStraight(this, container_)) ;
        }
    }

    @Override
    public void createTestAutoModes() {
    }

    @Override
    public void enableMessages() {
        MessageLogger.getTheMessageLogger().enableSubsystem(container_.getIntakeShooter().getName()) ;
        MessageLogger.getTheMessageLogger().enableSubsystem(container_.getDriveTrain().getName()) ;
        MessageLogger.getTheMessageLogger().enableSubsystem(container_.getTramp().getName()) ;
    }      

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        super.robotInit() ;

        Logger.disableDeterministicTimestamps();

        if (kLogToNetworkTables || XeroRobot.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter()) ;
        
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        
        Logger.start() ;
        
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        try {
            container_ = new AllegroContainer(this);
            setDriveController(container_.getController());
            enableMessages() ;

        } catch (Exception e) {
            MessageLogger logger = MessageLogger.getTheMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("exception caught creating robot container - " + e.getMessage()) ;
            logger.endMessage();
            logger.logStackTrace(e.getStackTrace());
        }
    }


    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        super.robotPeriodic();      

        if (!RobotConstants.kCharMode) {
            Logger.recordOutput("oi:buttons", container_.getDriveControllerOIString()) ;
            Logger.recordOutput("oi:rumble", getRumble());
        }
    }
}
