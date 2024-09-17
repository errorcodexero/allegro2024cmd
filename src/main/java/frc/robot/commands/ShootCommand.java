package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ShotType;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intakeshooter.IntakeShooterConstants;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.OILed;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveRotateToAngle;
import frc.robot.subsystems.tracker.TrackerSubsystem;

public class ShootCommand extends Command {
    private final static double kShootPositionTolerance = 2.0 ;
    private final static double kShootVelocityTolerance = 2.0 ;

    private OISubsystem oi_ ;
    private CommandSwerveDrivetrain db_ ;
    private IntakeShooterSubsystem intake_ ;
    private TrackerSubsystem tracker_ ;

    private SwerveRotateToAngle rotate_ ;
    private Command shoot_ ;

    public ShootCommand(OISubsystem oi, TrackerSubsystem tracker, CommandSwerveDrivetrain db, IntakeShooterSubsystem intake) {
        oi_ = oi ;
        tracker_ = tracker ;

        db_ = db ;
        intake_ = intake ;
        
        rotate_ = null ;      

        addRequirements();
        setName("shoot") ;
    }

    @Override
    public void initialize() {

        MessageLogger logger = MessageLogger.getTheMessageLogger() ;
        logger.startMessage(MessageType.Debug).add("Started The ShootCommand").endMessage();

        if (oi_.getShotType() == ShotType.Podium) {
            shoot_ = intake_.manualShootCommand(
                                IntakeShooterConstants.ManualShotPodium.kUpDownPos,
                                IntakeShooterConstants.ManualShotPodium.kUpDownPosTolerance,
                                IntakeShooterConstants.ManualShotPodium.kUpDownVelTolerance,
                                IntakeShooterConstants.ManualShotPodium.kTiltPos,
                                IntakeShooterConstants.ManualShotPodium.kTiltPosTolerance,
                                IntakeShooterConstants.ManualShotPodium.kTiltVelTolerance,
                                IntakeShooterConstants.ManualShotPodium.kShooterVel,
                                IntakeShooterConstants.ManualShotPodium.kShooterVelTolerance) ;
            CommandScheduler.getInstance().schedule(shoot_);
            rotate_ = null ;
        }
        else if (oi_.getShotType() == ShotType.Subwoofer) {
            shoot_ = intake_.manualShootCommand(
                                IntakeShooterConstants.ManualShotSubwoofer.kUpDownPos,
                                IntakeShooterConstants.ManualShotSubwoofer.kUpDownPosTolerance,
                                IntakeShooterConstants.ManualShotSubwoofer.kUpDownVelTolerance,
                                IntakeShooterConstants.ManualShotSubwoofer.kTiltPos,
                                IntakeShooterConstants.ManualShotSubwoofer.kTiltPosTolerance,
                                IntakeShooterConstants.ManualShotSubwoofer.kTiltVelTolerance,
                                IntakeShooterConstants.ManualShotSubwoofer.kShooterVel,
                                IntakeShooterConstants.ManualShotSubwoofer.kShooterVelTolerance) ;
            CommandScheduler.getInstance().schedule(shoot_);
            rotate_ = null ;
        }
        else {
            rotate_ = new SwerveRotateToAngle(db_, tracker_::angle)
                            .withPositionTolerance(kShootPositionTolerance)
                            .withVelocityTolerance(kShootVelocityTolerance) ;
            shoot_ = null ;
            CommandScheduler.getInstance().schedule(rotate_);
        }
    }

    @Override
    public void execute() {
        String str ="empty" ;

        boolean dbready = false ;

        if (rotate_ != null) {
            if (oi_.abort().getAsBoolean()) {
                rotate_.cancel();
                rotate_ = null ;
                str = "aborted" ;
            }
            else if (rotate_.isFinished()) {
                if (!tracker_.isFrozen()) {
                    tracker_.freezePose(true);                
                }
                if (tracker_.isOkToShoot()) {
                    rotate_ = null ;
                    shoot_ = intake_.shootCommand() ;
                    CommandScheduler.getInstance().schedule(shoot_);
                    str = "rfinished" ;
                }
                else {
                    str = "waitfortracker" ;
                }
            }
            else {
                str = "rotate" ;
            }
        }
        else if (shoot_ != null) {
            dbready = true ;
            if (intake_.hasShotLeft()) {
                tracker_.freezePose(false);
                str = "sfinished" ;
                shoot_ = null ;
            }
            else {
                str = "shoot" ;
            }
        }

        Logger.recordOutput("scmd-st", str) ;
        oi_.setLEDState(OILed.DBReady, dbready);
        oi_.setLEDState(OILed.ShooterReady, intake_.isShooterReady()) ;
        oi_.setLEDState(OILed.TiltReady, intake_.isTiltReady()) ;
        oi_.setLEDState(OILed.TrackerReady, tracker_.isOkToShoot()) ;
    }

    @Override
    public void end(boolean interrupted) {
        MessageLogger logger = MessageLogger.getTheMessageLogger() ;
        logger.startMessage(MessageType.Debug).add("end The ShootCommand", interrupted).endMessage();
    }

    @Override
    public boolean isFinished() {
        return rotate_ == null && shoot_ == null ;
    }
}
