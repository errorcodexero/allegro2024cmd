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

import org.xero1425.HolonomicPathFollower;
import org.xero1425.XeroContainer;
import org.xero1425.XeroRobot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        if (XeroRobot.isCompetition())
            db_ = TunerConstantsCompetition.DriveTrain ;
        else if (XeroRobot.isPractice())
            db_ = TunerConstantsPractice.DriveTrain ;
        else
            db_ = TunerConstantsSimulation.DriveTrain ;

        db_.createHolonimicPathFollower(getHolonomicConfig());

        tracker_ = new Tracker(robot, db_, limelight_name_) ;
        vision_ = new VisionSubsystem(robot, db_, limelight_name_) ;
        oi_ = new OISubsystem(robot, OIConstants.kOIControllerPort) ;

        intake_shooter_ = new IntakeShooterSubsystem(robot, () -> tracker_.distance(), ()-> oi_.getNoteDestination()) ;
        tramp_ = new TrampSubsystem(robot, ()-> oi_.getNoteDestination()) ;


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

        configureBindings();
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

    private void driveTrainBindings() {
        db_.setDefaultCommand(
            db_.applyRequest(() -> drive_.withVelocityX(-driver_controller_.getLeftY() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withVelocityY(-driver_controller_.getLeftX() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
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
    }

    private void configureBindings() {
        driveTrainBindings();
        superStructureBindings() ;
    }
}
