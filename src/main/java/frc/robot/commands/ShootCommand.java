package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.oi.OISubsystem;
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
    private SwerveRequest.SwerveDriveBrake brake_ ;    
    private Command shoot_ ;

    public ShootCommand(OISubsystem oi, TrackerSubsystem tracker, CommandSwerveDrivetrain db, IntakeShooterSubsystem intake) {
        oi_ = oi ;
        tracker_ = tracker ;

        db_ = db ;
        intake_ = intake ;

        addRequirements();
        setName("shoot") ;
    }

    @Override
    public void initialize() {
        brake_ = new SwerveRequest.SwerveDriveBrake() ;

        rotate_ = new SwerveRotateToAngle(db_, tracker_::angle)
                        .withPositionTolerance(kShootPositionTolerance)
                        .withVelocityTolerance(kShootVelocityTolerance) ;
        shoot_ = null ;
        
        CommandScheduler.getInstance().schedule(rotate_);
    }

    @Override
    public void execute() {
        if (rotate_ != null) {
            if (oi_.abort().getAsBoolean()) {
                rotate_.cancel();
                rotate_ = null ;
            }
            else if (rotate_.isFinished()) {
                rotate_ = null ;
                shoot_ = intake_.shootCommand() ;
                CommandScheduler.getInstance().schedule(shoot_);
            }
        }
        else if (shoot_ != null) {
            db_.setControl(new SwerveDriveBrake()) ;
        }
    }

    @Override
    public boolean isFinished() {
        return rotate_ == null && shoot_ == null ;
    }
}
