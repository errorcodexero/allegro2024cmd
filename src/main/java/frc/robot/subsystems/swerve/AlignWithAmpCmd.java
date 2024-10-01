package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.xero1425.math.Pose2dWithRotation;
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
    private static double kExtraSpacing = 1.0 ;

    private CommandSwerveDrivetrain dt_ ;
    private Pose2dWithRotation target_ ;
    private boolean has_target_ ;
    private AprilTagFieldLayout layout_ ;
    private State state_ ;
    private Pose2d start_ ;

    public AlignWithAmpCmd(AprilTagFieldLayout layout, CommandSwerveDrivetrain dt) {
        dt_ = dt ;
        layout_ = layout ;
        addRequirements(dt_);
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
            p = new Pose2d(p.getX(), p.getY() - robot_length_ / 2.0 - kExtraSpacing, p.getRotation()) ;
            target_ = new Pose2dWithRotation(p, p.getRotation()) ;
            has_target_ = true ;
        }
        else {
            has_target_ = false;
        }
    }

    @Override
    public void initialize() {
        computeDest() ;        
        dt_.driveTo("auto-amp", null, target_, 2.0, 2.0, 0.0, 0.5, 0.1) ;
        state_ = State.DriveToPoint ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case DriveToPoint:
                if (!dt_.isFollowingPath()) {
                    dt_.stopPath() ;
                    dt_.setControl(new ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(-0.5, 0.0, 0.0))) ;
                    start_ = dt_.getState().Pose ;
                    state_ = State.DrivingIn ;
                }
                break ;

            case DrivingIn:
                {
                    double dist = dt_.getState().Pose.getTranslation().getDistance(start_.getTranslation()); 
                    if (dist > kExtraSpacing) {
                        dt_.stopPath() ;
                        state_ = State.Done ;
                    }
                }
                break ;

            case Done:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || !has_target_ ;
    }
    
    @Override
    public void end(boolean interrupted) {
        dt_.stopPath() ;
        System.out.println("hello") ;
    }
}
