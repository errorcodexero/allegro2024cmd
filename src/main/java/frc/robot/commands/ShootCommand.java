package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.SwerveRotateToAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AllegroContainer;
import frc.robot.ShotType;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intakeshooter.IntakeShooterConstants;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.OILed;
import frc.robot.subsystems.tracker.TrackerSubsystem;

public class ShootCommand extends Command {
    
    private final static double kShootPositionDistanceNear = 1.0 ;
    private final static double kShootPositionDistanceFar = 4.0 ;
    private final static double kShootPositionToleranceAtFar = 3.0 ;
    private final static double kShootPositionToleranceAtNear = 3.0 ;
    private final static double kShootVelocityTolerance = 5.0 ;
 
    private AllegroContainer container_ ;
    private OISubsystem oi_ ;
    private CommandSwerveDrivetrain db_ ;
    private IntakeShooterSubsystem intake_ ;
    private TrackerSubsystem tracker_ ;

    private SwerveRotateToAngle rotate_ ;
    private Command shoot_ ;

    public ShootCommand(AllegroContainer c, OISubsystem oi, TrackerSubsystem tracker, CommandSwerveDrivetrain db, IntakeShooterSubsystem intake) {
        container_ = c ;
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

        shoot_ = null ;
        rotate_ = null ;

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
            container_.enableGamePad(false);
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
            container_.enableGamePad(false);                                
            CommandScheduler.getInstance().schedule(shoot_);
            rotate_ = null ;
        }
        else if (tracker_.isOkToShootAngleDistance()) {
            rotate_ = new SwerveRotateToAngle(db_, tracker_::angle)
                            .withPositionTolerance(rotatePositionTolerence())
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
            if (oi_.isAbortPressed()) {
                tracker_.freezePose(false);
                rotate_.cancel();
                rotate_ = null ;
                str = "aborted" ;
            }
            else if (rotate_.isFinished()) {
                container_.enableGamePad(false);
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
                    str = "exittracker" ;
                    shoot_ = null ;
                }
            }
            else {
                str = "rotate" ;
            }
        }
        else if (shoot_ != null) {
            dbready = true ;
            if (intake_.hasShotLeft()) {
                container_.enableGamePad(true) ;
                tracker_.freezePose(false);
                str = "sfinished" ;
                shoot_ = null ;
            }
            else {
                str = "shoot" ;
            }
        }

        Logger.recordOutput("state:shoot", str) ;
        oi_.setLEDState(OILed.DBReady, dbready);
    }

    @Override
    public void end(boolean interrupted) {
        // Just to be sure, this is always called when the command is done.
        container_.enableGamePad(true) ;        
    }

    @Override
    public boolean isFinished() {
        return rotate_ == null && shoot_ == null ;
    }

    //
    // Find the tolerance of the angle for rotating the drive base bassed on the distance to the target.
    //
    private double rotatePositionTolerence() {
        double delta = tracker_.distance() - kShootPositionDistanceNear ;
        double percent = delta / (kShootPositionDistanceFar - kShootPositionDistanceNear) ;
        double output = kShootPositionToleranceAtFar - percent * (kShootPositionToleranceAtFar - kShootPositionToleranceAtNear) ;
        return output ;
    }       
}
