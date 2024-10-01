package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignWithAmpCmd extends Command {

    private enum State {
        DriveToPoint,
        DrivingIn,
        Done
    }

    private static double robot_length_ = 0.9779 ;
    private static double kExtraSpacing = 0.01 ;
    private static double kMaxDistance = 3.0 ;

    private CommandSwerveDrivetrain db_ ;
    private Pose2dWithRotation target_ ;
    private boolean has_target_ ;
    private AprilTagFieldLayout layout_ ;
    private State state_ ;
    private Pose2d start_ ;

    public AlignWithAmpCmd(AprilTagFieldLayout layout, CommandSwerveDrivetrain dt) {
        db_ = dt ;
        layout_ = layout ;
        addRequirements(db_);
    }

    @Override
    public void initialize() {
        computeDest() ;        
        db_.driveTo("auto-amp", null, target_, 2.0, 2.0, 0.0, 0.5, 0.1) ;
        state_ = State.DriveToPoint ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case DriveToPoint:
                if (!db_.isFollowingPath()) {
                    db_.stopPath() ;
                    db_.setControl(new ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(-0.5, 0.0, 0.0))) ;
                    start_ = db_.getState().Pose ;
                    state_ = State.DrivingIn ;
                }
                break ;

            case DrivingIn:
                {
                    double dist = db_.getState().Pose.getTranslation().getDistance(start_.getTranslation()); 
                    if (dist > kExtraSpacing) {
                        db_.stopPath() ;
                        state_ = State.Done ;
                    }
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("states:auto-amp", state_) ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || !has_target_ ;
    }
    
    @Override
    public void end(boolean interrupted) {
        db_.stopPath() ;
    }

    private void computeDest() {
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
                has_target_ = true ;
            }
            else {
                has_target_ = false ;
            }
        }
        else {
            has_target_ = false;
        }
    }    
}
