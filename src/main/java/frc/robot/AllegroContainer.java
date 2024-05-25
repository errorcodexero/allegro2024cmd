package frc.robot;

import frc.robot.commands.TransferNoteCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.generated.TunerConstantsCompetition;
import frc.robot.generated.TunerConstantsPractice;
import frc.robot.generated.TunerConstantsSimulation;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.oi.OIConstants;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveRotateToAngle;
import frc.robot.subsystems.tracker.Tracker;
import frc.robot.subsystems.tramp.TrampSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.Supplier;

import org.xero1425.HolonomicPathFollower;
import org.xero1425.MessageLogger;
import org.xero1425.MessageType;
import org.xero1425.XeroContainer;
import org.xero1425.XeroRobot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private final TrampSubsystem tramp_  ;
    private final Tracker tracker_ ;
    private final VisionSubsystem vision_ ;
    private final OISubsystem oi_ ;

    //
    // Limelight name
    //
    private final String limelight_name_ = "limelight" ;

    //
    // OI related devices
    //
    private final CommandXboxController driver_controller_ ;

    //
    // Telemetry related
    //
    private final Telemetry logger_ = new Telemetry(TunerConstantsCompetition.kSpeedAt12VoltsMps);

    //
    // Commands
    //
    private final SwerveRequest.FieldCentric drive_ = new SwerveRequest.FieldCentric()
                                                            .withDeadband(TunerConstantsCompetition.kSpeedAt12VoltsMps * 0.1)
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
        if (robot.isPracticeBot()) {
            db_ = TunerConstantsPractice.DriveTrain ;
        }
        else if (robot.isCompetitionBot()) {
            db_ = TunerConstantsCompetition.DriveTrain ;
        }
        else if (RobotBase.isSimulation()) {
            db_ = TunerConstantsSimulation.DriveTrain ;
        }
        else {
            //
            // This should never happen, throw an exception if it does
            //
            MessageLogger logger = robot.getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("the robot is not valid, it is neither a practice bot, a competition bot, or a simulation") ;
            logger.endMessage();

            throw new Exception("invalid robot") ;
        }

        db_.createHolonimicPathFollower(getHolonomicConfig());

        Supplier<NoteDestination> notesupply = null ;

        if (!robot.isCharMode()) {
            //
            // We are not characterizing.  Create the OI subsystem,
            //
            oi_ = new OISubsystem(robot, OIConstants.kOIControllerPort) ;
            notesupply = () -> oi_.getNoteDestination() ;
        }
        else {
            //
            // Characterization is triggered all by the game pad.
            // Assign the currently active mechanisms within a subsystem that is to be characterized
            // to the gamepad.
            //
            oi_ = null ;
        }

        tracker_ = new Tracker(robot, db_, limelight_name_) ;
        vision_ = new VisionSubsystem(robot, db_, limelight_name_) ;
        intake_shooter_ = new IntakeShooterSubsystem(robot, () -> tracker_.distance(), notesupply) ;
        tramp_ = new TrampSubsystem(robot, notesupply) ;

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
        driver_controller_ = new CommandXboxController(OIConstants.kDriverControllerPort);

        configureBindings(robot);
    }

    public HolonomicPathFollower.Config getHolonomicConfig() {
        HolonomicPathFollower.Config cfg = new HolonomicPathFollower.Config() ;

        cfg.max_rot_velocity = SwerveConstants.kMaxRotationalSpeed ;
        cfg.max_rot_acceleration = SwerveConstants.kMaxRotationalAccel ;

        cfg.rot_p = RobotConstants.PathFollowing.RotCtrl.kP ;
        cfg.rot_i = RobotConstants.PathFollowing.RotCtrl.kI ;
        cfg.rot_d = RobotConstants.PathFollowing.RotCtrl.kD ;

        cfg.x_d = RobotConstants.PathFollowing.XCtrl.kD ;
        cfg.x_i = RobotConstants.PathFollowing.XCtrl.kI ;
        cfg.x_p = RobotConstants.PathFollowing.XCtrl.kP ;

        cfg.y_d = RobotConstants.PathFollowing.YCtrl.kD ;
        cfg.y_i = RobotConstants.PathFollowing.YCtrl.kI ;
        cfg.y_p = RobotConstants.PathFollowing.YCtrl.kP ;

        cfg.xytolerance = RobotConstants.PathFollowing.kXYTolerance ;
        cfg.rot_tolerance = Rotation2d.fromDegrees(RobotConstants.PathFollowing.kAngleTolerance) ;

        cfg.pose_supplier = () -> getDriveTrain().getState().Pose ;
        cfg.output_consumer = (ChassisSpeeds spd) -> setDriveTrainChassisSpeeds(spd) ;

        return cfg ;
    }

    private void setDriveTrainChassisSpeeds(ChassisSpeeds spd) {
        getDriveTrain().setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ;
    }

    public OISubsystem getOI() {
        return oi_ ;
    }

    public IntakeShooterSubsystem getIntakeShooter() {
        return intake_shooter_ ;
    }

    public TrampSubsystem getTramp() {
        return tramp_ ;
    }

    public CommandSwerveDrivetrain getDriveTrain() {
        return db_ ;
    }

    private void charBindings() throws Exception {

        int total = 0 ;

        if (RobotConstants.WhichSubsystem.kCharDBSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(db_.sysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(db_.sysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(db_.sysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(db_.sysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharUpDownSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(intake_shooter_.upDownSysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(intake_shooter_.upDownSysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(intake_shooter_.upDownSysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(intake_shooter_.upDownSysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharTiltSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(intake_shooter_.tiltSysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(intake_shooter_.tiltSysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(intake_shooter_.tiltSysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(intake_shooter_.tiltSysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharShooter1Subsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(intake_shooter_.shooter1SysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(intake_shooter_.shooter1SysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(intake_shooter_.shooter1SysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(intake_shooter_.shooter1SysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharShooter2Subsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(intake_shooter_.shooter2SysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(intake_shooter_.shooter2SysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(intake_shooter_.shooter2SysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(intake_shooter_.shooter2SysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if(RobotConstants.WhichSubsystem.kCharElevatorSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(tramp_.elevatorSysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(tramp_.elevatorSysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(tramp_.elevatorSysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(tramp_.elevatorSysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharArmSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(tramp_.armSysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(tramp_.armSysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(tramp_.armSysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(tramp_.armSysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharClimberSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(tramp_.climberSysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(tramp_.climberSysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(tramp_.climberSysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(tramp_.climberSysIdDynamic(Direction.kReverse));
            total++ ;
        }

        if (RobotConstants.WhichSubsystem.kCharManipulatorSubsystem) {
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.x()).whileTrue(tramp_.manipulatorSysIdQuasistatic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.y()).whileTrue(tramp_.manipulatorSysIdQuasistatic(Direction.kReverse));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.a()).whileTrue(tramp_.manipulatorSysIdDynamic(Direction.kForward));
            driver_controller_.leftBumper().and(driver_controller_.rightBumper()).and(driver_controller_.b()).whileTrue(tramp_.manipulatorSysIdDynamic(Direction.kReverse));
            total++ ;
        }        

        if (total > 1) {
            throw new Exception("Only one subsystem can be characterized at a time") ;
        }
    }

    private void driveTrainBindings() {
        db_.setDefaultCommand(
            db_.applyRequest(() -> drive_.withVelocityX(-driver_controller_.getLeftY() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withVelocityY(-driver_controller_.getLeftX() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withRotationalRate(-driver_controller_.getRightX() * SwerveConstants.kMaxRotationalSpeed)
                            ).ignoringDisable(true));

        driver_controller_.y().and(driver_controller_.b()).onTrue(db_.runOnce(()->db_.seedFieldRelative())) ;

        db_.registerTelemetry(logger_::telemeterize) ;
    }

    private void superStructureBindings() {
        //
        // Collect command, bound to OI and the gamepad
        //
        driver_controller_.rightBumper().or(oi_.collect()).whileTrue(intake_shooter_.collectCommand()) ;

        //
        // Eject command, bound to the eject button on the OI
        //
        oi_.eject().onTrue(new ParallelCommandGroup(intake_shooter_.ejectCommand(), tramp_.ejectCommand())) ;

        //
        // Turtle command, bound to the turtle button on the OI
        //
        oi_.turtle().onTrue(new ParallelCommandGroup(intake_shooter_.turtleCommand(), tramp_.turtleCommand())) ;

        //
        // Shoot command, bound to the shoot button on the OI and only targeting the intake
        //
        oi_.shoot().and(intake_shooter_.readyForShoot()).onTrue(rotate_.andThen(intake_shooter_.shootCommand())) ;

        //
        // Shoot command, bound to the shoot button on the OI and only targeting the tramp (AMP)
        //
        oi_.shoot().and(tramp_.readyForAmp()).onTrue(tramp_.shootCommand()) ;

        //
        // Climb Up Exec, bound to complete the trap sequence
        //
        oi_.climbUpExec().and(tramp_.readyForTrap()).onTrue(tramp_.trapCommand()) ;

        //
        // If a note is collected and thet arget is the trap or amp, this trigger is fired to complete
        // the transfer action.  The transfer action moves the note from the intake to the manipulator.
        //
        intake_shooter_.readyForTransferNote().onTrue(new TransferNoteCommand(intake_shooter_, tramp_)) ;

        oi_.climbUpPrep().and(tramp_.isClimberDown()).onTrue(tramp_.climberUpCmd()) ;
        oi_.climbUpExec().and(tramp_.isBasicClimbReady()).onTrue(tramp_.basicClimbCmd()) ;
    }

    private void configureBindings(XeroRobot robot) throws Exception {
        if (robot.isCharMode()) {
            charBindings() ;
        }
        else {
            driveTrainBindings();
            superStructureBindings() ;
        }
    }
}
