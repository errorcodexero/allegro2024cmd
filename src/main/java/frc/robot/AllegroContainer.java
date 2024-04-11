// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.OIConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.TrapArmSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import org.xero1425.XeroContainer;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link AllegroRobot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class AllegroContainer extends XeroContainer {

    //
    // Subsystems
    //
    private final CommandSwerveDrivetrain db_ ;
    private final IntakeShooterSubsystem intake_shooter_ ;
    private final TrapArmSubsystem trap_arm_  ;

    //
    // OI related devices
    //
    private final CommandGenericHID oi_ ;
    private final CommandXboxController driver_controller_ ;

    //
    // Telemetry related
    //
    private final Telemetry logger_ = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    //
    // Commands
    //
    private final SwerveRequest.FieldCentric drive_ = new SwerveRequest.FieldCentric()
                                                            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1)
                                                            .withRotationalDeadband(SwerveConstants.kMaxRotationalSpeed * 0.1)
                                                            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     * @throws Exception 
     */
    public AllegroContainer(AllegroRobot robot) throws Exception {
        super(robot) ;
        
        //
        // Create subsystems
        //
        db_ = TunerConstants.DriveTrain ;
        intake_shooter_ = new IntakeShooterSubsystem(robot, null) ;
        trap_arm_ = new TrapArmSubsystem(robot) ;

        //
        // Create OI devices
        //
        oi_ = new CommandGenericHID(OIConstants.kOIControllerPort) ;
        driver_controller_ = new CommandXboxController(OIConstants.kDriverControllerPort);

        configureBindings();
    }

    public IntakeShooterSubsystem getIntakeShooter() {
        return intake_shooter_ ;
    }

    public TrapArmSubsystem getTrapArm() {
        return trap_arm_ ;
    }

    public CommandSwerveDrivetrain getDriveTrain() {
        return db_ ;
    }

    private void driveTrainBindings() {
        db_.setDefaultCommand(
            db_.applyRequest(() -> drive_.withVelocityX(-driver_controller_.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                         .withVelocityY(-driver_controller_.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                         .withRotationalRate(-driver_controller_.getRightX() * SwerveConstants.kMaxRotationalSpeed)
                            ).ignoringDisable(true));

        driver_controller_.a().onTrue(db_.runOnce(()->db_.seedFieldRelative())) ;

        db_.registerTelemetry(logger_::telemeterize) ;

        driver_controller_.back().and(driver_controller_.y()).whileTrue(db_.sysIdDynamic(Direction.kForward));
        driver_controller_.back().and(driver_controller_.x()).whileTrue(db_.sysIdDynamic(Direction.kReverse));
        driver_controller_.start().and(driver_controller_.y()).whileTrue(db_.sysIdQuasistatic(Direction.kForward));
        driver_controller_.start().and(driver_controller_.x()).whileTrue(db_.sysIdQuasistatic(Direction.kReverse));        
    }

    private void superStructureBindings() {
        driver_controller_.rightBumper().or(oi_.button(OIConstants.Buttons.kCollect)).whileTrue(intake_shooter_.collectCommand()) ;
        oi_.button(OIConstants.Buttons.kEject).onTrue(new ParallelCommandGroup(intake_shooter_.ejectCommand(), trap_arm_.ejectCommand())) ;
    }

    private void configureBindings() {
        driveTrainBindings();
        superStructureBindings() ;
    }

    public Command getAutonomousCommand() {
        return null ;
    }

    public IntakeShooterSubsystem getIntakeShooterSubsystem() {
        return intake_shooter_ ;
    }

    public TrapArmSubsystem getTrapArmSubsystem() {
        return trap_arm_ ;
    }
}
