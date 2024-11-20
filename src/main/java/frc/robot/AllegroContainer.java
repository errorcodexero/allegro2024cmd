package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.xero1425.base.XeroContainer;
import org.xero1425.base.XeroRobot;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.SwerveConstants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ConditionalVibrateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstantsCompetition;
import frc.robot.subsystems.gamepiecetracker.GamePieceTracker;
import frc.robot.subsystems.intakeshooter.IntakeShooterConstants;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.oi.OIConstants;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tracker.TrackerSubsystem;

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
    // private static int kAutoTrap = 2 ;
    
    // #region private member variables
    //
    // Subsystems
    //
    private CommandSwerveDrivetrain db_ ;
    private IntakeShooterSubsystem intake_shooter_ ;
    private TrackerSubsystem tracker_ ;
    private GamePieceTracker game_piece_tracker_ ;
    private OISubsystem oi_ ;

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    //
    // Limelight name
    //
    public final String limelight_name_ = "limelight" ;

    //
    // OI related devices
    //
    private final CommandXboxController driver_controller_ ;
    private boolean driver_controller_enabled_ ;

    //
    // Telemetry related
    //
    private final Telemetry logger_ = new Telemetry(TunerConstantsCompetition.kSpeedAt12VoltsMps);

    //
    // Commands
    //
    private final SwerveRequest.FieldCentric drive_ = new SwerveRequest.FieldCentric()
                                                            .withDeadband(TunerConstantsCompetition.kSpeedAt12VoltsMps * 0.05)
                                                            .withRotationalDeadband(SwerveConstants.kMaxRotationalSpeed * 0.05)
                                                            .withDriveRequestType(DriveRequestType.Velocity);        

    private final SwerveRequest.SwerveDriveBrake brake_ = new SwerveRequest.SwerveDriveBrake() ;

    final private double SlowFactor = 0.1 ;

    // #endregion

    // #region constructor
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     * @throws Exception
     */
    public AllegroContainer(AllegroRobot robot) throws Exception {
        super(robot) ;

        driver_controller_enabled_ = true ;

        //
        // Create subsystems
        //


        Supplier<NoteDestination> notesupply = null ;
        Supplier<ShotType> shotsupply = null ;

        if (!robot.isCharMode()) {
            //
            // We are not characterizing.  Create the OI subsystem,
            //
            oi_ = new OISubsystem(robot, OIConstants.kOIControllerPort) ;
            notesupply = () -> oi_.getNoteDestination() ;
            shotsupply = () -> oi_.getShotType() ;
        }
        else {
            //
            // Characterization is triggered all by the game pad.
            // Assign the currently active mechanisms within a subsystem that is to be characterized
            // to the gamepad.
            //
            oi_ = null ;
        }

        // tramp_ = new TrampSubsystem(robot, notesupply) ;        

        tracker_ = new TrackerSubsystem(robot, db_, limelight_name_) ;
        intake_shooter_ = new IntakeShooterSubsystem(robot, () -> tracker_.distance(), notesupply, shotsupply) ;

        game_piece_tracker_ = new GamePieceTracker(robot, "limelight-note", 0.5, 33.6, -20.0) ;

        db_ = new CommandSwerveDrivetrain(robot, TunerConstantsCompetition.DrivetrainConstants, 
                                                TunerConstantsCompetition.FrontLeft, 
                                                TunerConstantsCompetition.FrontRight, 
                                                TunerConstantsCompetition.BackLeft, 
                                                TunerConstantsCompetition.BackRight);        


        if (db_ != null) {
            db_.setLimelightName(limelight_name_);
        }        

        


        //
        // Create OI devices
        //
        driver_controller_ = new CommandXboxController(OIConstants.kDriverControllerPort);

        configureBindings(robot);
    }
    // #endregion

    public void enableGamePad(boolean b) {
        driver_controller_enabled_ = b ;
    }

    public XboxController getController() {
        return driver_controller_.getHID() ;
    }

    // #region get subsytems
    public OISubsystem getOI() {
        return oi_ ;
    }

    public IntakeShooterSubsystem getIntakeShooter() {
        return intake_shooter_ ;
    }

    public GamePieceTracker getGamePieceTracker() {
        return game_piece_tracker_ ;
    }

    // public TrampSubsystem getTramp() {
    //     return tramp_ ;
    // }

    public CommandSwerveDrivetrain getDriveTrain() {
        return db_ ;
    }

    public TrackerSubsystem getTracker() {
        return tracker_ ;
    }
    // #endregion

    // #region Drive Train Bindings
    private double getLeftX() {
        if (!driver_controller_enabled_)
            return 0.0 ;

        double y = -driver_controller_.getLeftX() ;

        if (driver_controller_.getHID().getXButton()) {
            y = y * SlowFactor ;
        }
        else { 
            y = Math.signum(y) * y * y ;
        }
        
        return y ;
    }

    private double getLeftY() {
        if (!driver_controller_enabled_)
            return 0.0 ;

        double x = -driver_controller_.getLeftY() ;

        if (driver_controller_.getHID().getXButton()) {
            x = x * SlowFactor ;
        }
        else {
            x = Math.signum(x) * x * x;
        }

        return x ;
    }

    private double getRightX() {
        if (!driver_controller_enabled_)
            return 0.0 ;

        double x = -driver_controller_.getRightX() ;

        if (driver_controller_.getHID().getXButton()) {
            x = x * SlowFactor ;
        }
        else {
            x = Math.signum(x) * x * x  ;
        }           
        return x ;
    }

    private void yandbPressed() {
        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            Pose2d pose  ;
            if (alliance.get() == Alliance.Red) {
                pose = new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)) ;
            }
            else {
                pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)) ;                
            }
            db_.seedFieldRelative(pose) ;
        }
        else {
            DriverStation.reportError("Gamepad Y & B pressed before alliance is known (should be impossible)", false) ;
        }
    }

    private void driveTrainBindings() {
        db_.setDefaultCommand(
            db_.applyRequest(() -> drive_.withVelocityX(getLeftY() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withVelocityY(getLeftX() * TunerConstantsCompetition.kSpeedAt12VoltsMps)
                                         .withRotationalRate(getRightX() * SwerveConstants.kMaxRotationalSpeed), "drive"
                            ).ignoringDisable(true));

        driver_controller_.y().and(driver_controller_.b()).onTrue(db_.runOnce(()->yandbPressed()).ignoringDisable(true)) ;

        driver_controller_.leftBumper().whileTrue(db_.applyRequest(() -> brake_, "brake").ignoringDisable(true)) ;
        driver_controller_.pov(0).whileTrue(db_.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0), "pov0")) ;
        driver_controller_.pov(90).whileTrue(db_.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(-0.5), "pov90")) ;        
        driver_controller_.pov(180).whileTrue(db_.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0), "pov180")) ;
        driver_controller_.pov(270).whileTrue(db_.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.5), "pov270")) ;          

        db_.registerTelemetry(logger_::telemeterize) ;
    }
    // #endregion

    // #region superstructure bindings

    private void superStructureBindings() {
        //
        // Collect command, bound to OI and the gamepad
        //
        BooleanSupplier condition = () -> intake_shooter_.hasNote() ;
        driver_controller_.rightBumper().or(oi_.collect()).whileTrue(intake_shooter_.collectCommand().andThen(new ConditionalVibrateCommand(getRobot(), 1.5, condition))) ;

        //
        // Eject command, bound to the eject button on the OI
        //
        oi_.eject().onTrue(intake_shooter_.ejectCommand()) ;

        //
        // Turtle command, bound to the turtle button on the OI
        //
        oi_.turtle().onTrue(intake_shooter_.turtleCommand()) ;

        //
        // Shoot command, bound to the shoot button on the OI and only targeting the intake
        //
        oi_.shoot().or(driver_controller_.a()).and(intake_shooter_.readyToShoot()).onTrue(new ShootCommand(this, oi_, tracker_, db_, intake_shooter_)) ;

        // //
        // // Shoot command, bound to the shoot button on the OI and only targeting the tramp (AMP)
        // //
        // driver_controller_.a().or(oi_.shoot()).and(tramp_.readyForAmp()).onTrue(tramp_.shootCommand()) ;

        // //
        // // Climb Up Exec, bound to complete the trap sequence
        // //
        // oi_.climbUpExec().and(tramp_.readyForTrap()).onTrue(tramp_.trapCommand()) ;

        // //
        // // If a note is collected and the target is the trap or amp, this trigger is fired to complete
        // // the transfer action.  The transfer action moves the note from the intake to the manipulator.
        // //
        // intake_shooter_.readyForTransferNote().onTrue(new TransferNoteCommand(db_, intake_shooter_, tramp_)) ;

        // oi_.climbUpPrep().and(tramp_.isClimberDown()).onTrue(tramp_.climberUpCmd()) ;
        // oi_.climbUpExec().and(tramp_.isBasicClimbReady()).onTrue(tramp_.basicClimbCmd()) ;

        Command ferry = intake_shooter_.manualShootCommand(
                IntakeShooterConstants.ManualShotFerry.kUpDownPos,
                IntakeShooterConstants.ManualShotFerry.kUpDownPosTolerance,
                IntakeShooterConstants.ManualShotFerry.kUpDownVelTolerance,
                IntakeShooterConstants.ManualShotFerry.kTiltPos,
                IntakeShooterConstants.ManualShotFerry.kTiltPosTolerance,
                IntakeShooterConstants.ManualShotFerry.kTiltVelTolerance,
                IntakeShooterConstants.ManualShotFerry.kShooterVel,
                IntakeShooterConstants.ManualShotFerry.kShooterVelTolerance) ;
        driver_controller_.leftTrigger().onTrue(ferry) ;
    }
    // #endregion

    private void configureBindings(XeroRobot robot) throws Exception {
        if (db_ != null) {
            driveTrainBindings();
        }
        superStructureBindings() ;
    }
    // #endregion

    // #region misc methods
    private String getDriveControllerString() {
        String str = "" ;

        if (driver_controller_.getHID().getAButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "a" ;
        }

        if (driver_controller_.getHID().getBButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "b" ;
        }        

        if (driver_controller_.getHID().getXButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "x" ;
        }    
        
        if (driver_controller_.getHID().getYButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "y" ;
        }   
        
        if (driver_controller_.getHID().getBackButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "back" ;
        }      
        
        if (driver_controller_.getHID().getStartButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "start" ;
        }   
        
        if (driver_controller_.getHID().getLeftBumper()) {
            if (str.length() > 0)
                str += "," ;
            str += "lb" ;
        }     
        
        if (driver_controller_.getHID().getRightBumper()) {
            if (str.length() > 0)
                str += "," ;
            str += "rb" ;
        }          

        if (driver_controller_.getHID().getLeftStickButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "ls" ;
        }     
        
        if (driver_controller_.getHID().getRightStickButton()) {
            if (str.length() > 0)
                str += "," ;
            str += "rs" ;
        }    

        return str ;
    }

    public String getDriveControllerOIString() {
        String str = getDriveControllerString() ;

        String oistr = oi_.getPressedString() ;
        if (oistr.length() > 0 && str.length() > 0)
            str += "," ;
        str += oistr ;

        str = "[" + str + "]";
        return str ;
    }    

    // #endregion
}
