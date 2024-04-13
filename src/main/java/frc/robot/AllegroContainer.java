package frc.robot;

import frc.robot.commands.TransferNoteCommand;
import frc.robot.constants.OIConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveRotateToAngle;
import frc.robot.subsystems.tracker.Tracker;
import frc.robot.subsystems.tramp.TrampSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

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
    private final TrampSubsystem trap_arm_  ;
    private final Tracker tracker_ ;
    private final VisionSubsystem vision_ ;

    //
    // Limelight name
    //
    private final String limelight_name_ = "limelight" ;

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

    private final SwerveRotateToAngle rotate_ ;

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

        tracker_ = new Tracker(robot, db_, limelight_name_) ;
        vision_ = new VisionSubsystem(robot, db_, limelight_name_) ;
        intake_shooter_ = new IntakeShooterSubsystem(robot, () -> tracker_.distance()) ;
        trap_arm_ = new TrampSubsystem(robot) ;

        vision_.enable(true);

        //
        // Create commands
        //
        rotate_ = new SwerveRotateToAngle(db_, tracker_::angle)
                    .withPositionTolerance(SwerveConstants.kShootPositionTolerance)
                    .withVelocityTolerance(SwerveConstants.kShootVelocityTolerance) ;

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

    public TrampSubsystem getTrapArm() {
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
        //
        // Collect command, bound to OI and the gamepad
        //
        // driver_controller_.rightBumper().or(oi_.button(OIConstants.Buttons.kCollect)).whileTrue(intake_shooter_.collectCommand()) ;
        driver_controller_.rightBumper().whileTrue(intake_shooter_.collectCommand()) ;

        //
        // Eject command, bound to the eject button on the OI
        //
        oi_.button(OIConstants.Buttons.kEject).onTrue(new ParallelCommandGroup(intake_shooter_.ejectCommand(), trap_arm_.ejectCommand())) ;

        //
        // Turtle command, bound to the turtle button on the OI
        // 
        oi_.button(OIConstants.Buttons.kTurtle).onTrue(new ParallelCommandGroup(intake_shooter_.turtleCommand(), trap_arm_.turtleCommand())) ;

        //
        // Set the target based on changes to the target switch.  This tells the intake and tramp where to send the note. If the subsystems do not have a
        // note, the change in destination is just recorded.
        //
        oi_.button(OIConstants.Buttons.kTarget1).and(oi_.button(OIConstants.Buttons.kTarget2).negate()).onTrue(
                    new ParallelCommandGroup(intake_shooter_.targetSpeakerCommand(), trap_arm_.targetSpeakerCommand())) ;
        oi_.button(OIConstants.Buttons.kTarget1).negate().and(oi_.button(OIConstants.Buttons.kTarget2).negate()).onTrue(
                    new ParallelCommandGroup(intake_shooter_.targetAmpCommand(), trap_arm_.targetAmpCommand())) ;
        oi_.button(OIConstants.Buttons.kTarget1).negate().and(oi_.button(OIConstants.Buttons.kTarget2)).onTrue(
                    new ParallelCommandGroup(intake_shooter_.targetTrapCommand(), trap_arm_.targetTrapCommand())) ;

        //
        // Shoot command, bound to the shoot button on the OI
        //
        oi_.button(OIConstants.Buttons.kShoot).onTrue(rotate_.andThen(intake_shooter_.shootCommand())) ;
        // oi_.button(OIConstants.Buttons.kAbort).onTrue(abort_shoot_) ;

        //
        // If a note is collected and thet arget is the trap or amp, this trigger is fired to complete
        // the transfer action.  The transfer action moves the note from the intake to the manipulator.
        //
        intake_shooter_.getTransferNoteTrigger().onTrue(new TransferNoteCommand(intake_shooter_, trap_arm_)) ;
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

    public TrampSubsystem getTrapArmSubsystem() {
        return trap_arm_ ;
    }
}
