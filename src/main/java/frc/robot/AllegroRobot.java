// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SimArgs;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.automodes.competition.DriveStraight;
import frc.robot.automodes.competition.FourNoteDynamicCommand;
import frc.robot.automodes.testmodes.TeeTestModeCommand;
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
    private AllegroContainer container_;

    public AllegroRobot() {
        super(OIConstants.kDriverControllerPort, OIConstants.kOIControllerPort) ;
        setFieldLayout(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()) ;

        logSubsystemCycles(0) ;
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

        return "transfer";
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
        addAutoMode(new FourNoteDynamicCommand(this, container_));
        addAutoMode(new DriveStraight(this, container_));
    }

    @Override
    public void createTestAutoModes() {
        addAutoMode(new TeeTestModeCommand(this, container_)) ;
    }    

    @Override
    public void enableMessages() {
        getMessageLogger().enableSubsystem(container_.getIntakeShooter().getName()) ;
        getMessageLogger().enableSubsystem(container_.getDriveTrain().getName()) ;
        getMessageLogger().enableSubsystem(container_.getTramp().getName()) ;
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

        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables         
        Logger.start() ;

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        try {
            container_ = new AllegroContainer(this);
            enableMessages() ;

        } catch (Exception e) {
            MessageLogger logger = getMessageLogger() ;
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
        
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();      
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link AllegroContainer} class.
     */
    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        super.simulationInit();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
