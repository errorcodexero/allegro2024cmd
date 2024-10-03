package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.SwerveRotateToAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.OILed;
import frc.robot.subsystems.tracker.TrackerSubsystem;

public class AutoShootCommand extends Command {
    
    private final static double kShootPositionDistanceNear = 1.0 ;
    private final static double kShootPositionDistanceFar = 4.0 ;
    private final static double kShootPositionToleranceAtFar = 3.0 ;
    private final static double kShootPositionToleranceAtNear = 3.0 ;
    private final static double kShootVelocityTolerance = 5.0 ;
 

    private OISubsystem oi_ ;
    private CommandSwerveDrivetrain db_ ;
    private IntakeShooterSubsystem intake_ ;
    private TrackerSubsystem tracker_ ;

    private SwerveRotateToAngle rotate_ ;
    private Command shoot_ ;

    public AutoShootCommand(OISubsystem oi, TrackerSubsystem tracker, CommandSwerveDrivetrain db, IntakeShooterSubsystem intake) {
        oi_ = oi ;
        tracker_ = tracker ;

        db_ = db ;
        intake_ = intake ;
        
        rotate_ = null ;      
        setName("shoot") ;
    }

    @Override
    public void initialize() {
        rotate_ = new SwerveRotateToAngle(db_, tracker_::angle)
                            .withPositionTolerance(rotatePositionTolerence())
                            .withVelocityTolerance(kShootVelocityTolerance) ;
        shoot_ = null ;
        CommandScheduler.getInstance().schedule(rotate_);
    }

    @Override
    public void execute() {
        String str ="empty" ;

        boolean dbready = false ;

        if (rotate_ != null) {
            if (rotate_.isFinished()) {
                if (!tracker_.isFrozen()) {
                    tracker_.freezePose(true);
                }
                if (tracker_.isOkToShoot()) {
                    rotate_ = null ;
                    shoot_ = intake_.shootCommand() ;
                    CommandScheduler.getInstance().schedule(shoot_);
                    str = "rfinished" ;
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

        Logger.recordOutput("states:autoshoot", str) ;
        oi_.setLEDState(OILed.DBReady, dbready);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        boolean ret = (rotate_ == null && shoot_ == null) ;
        return ret ;
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
