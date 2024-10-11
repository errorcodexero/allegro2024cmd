
package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class AutoAmp extends Command {

    private enum State {
        Starting,
        DriveToPoint,
        Shooting, 
        Done
    }

    private static double robot_length_ = 0.9779 ;
    private static double kExtraSpacing = -0.04 ;
    private static double kMaxDistance = 3.0 ;

    private OISubsystem oi_ ;    
    private TrampSubsystem tramp_ ;
    private CommandSwerveDrivetrain db_ ;
    private Pose2dWithRotation target_ ;
    private AprilTagFieldLayout layout_ ;
    private State state_ ;

    public AutoAmp(AprilTagFieldLayout layout, OISubsystem oi, TrampSubsystem tramp, CommandSwerveDrivetrain dt) {
        db_ = dt ;
        oi_ = oi ;
        tramp_ = tramp ;
        layout_ = layout ;
        state_ = State.Starting ;
        addRequirements(db_);
    }

    @Override
    public void initialize() {
        if (computeTarget()) {
            db_.driveTo("auto-amp", null, target_, 2.0, 2.0, 0.0, 1.0, 0.1) ;
            state_ = State.DriveToPoint ;
        }
        else {
            state_ = State.Done ;
        }
    }

    @Override
    public void execute() {

        switch(state_) {
            case Starting:
                break ;

            case DriveToPoint:
                if (oi_.isAbortPressed()) {
                    db_.stopPath() ;
                    state_ = State.Done ;
                    return ;
                }            
                else if (!db_.isFollowingPath()) {
                    db_.stopPath() ;
                    tramp_.shoot();
                    state_ = State.Shooting ;
                }
                break ;

            case Shooting:
                if (!tramp_.isShooting()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("states:auto-amp", state_) ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
    
    @Override
    public void end(boolean interrupted) {
        db_.stopPath() ;
    }

    private boolean computeTarget() {
        boolean ret = false ;

        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            int tag = -1 ;

            if (alliance.get() == Alliance.Red) {
                tag = 5 ;
            }
            else {
                tag = 6 ;
            }

            Pose2d p = layout_.getTagPose(tag).get().toPose2d() ;
            p = new Pose2d(p.getX(), p.getY() - robot_length_ / 2.0 - kExtraSpacing, Rotation2d.fromDegrees(90.0)) ;
            target_ = new Pose2dWithRotation(p, Rotation2d.fromDegrees(-90.0)) ;

            double dist = target_.getTranslation().getDistance(db_.getState().Pose.getTranslation()) ;
            if (dist < kMaxDistance) {
                ret = true ;
            }
        }

        return ret;
    }    
}
