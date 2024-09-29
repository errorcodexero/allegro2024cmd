package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignWithAmpCmd extends Command {
    private static double robot_length_ = 0.8 ;

    private CommandSwerveDrivetrain dt_ ;
    private Pose2dWithRotation target_ ;
    private boolean has_target_ ;

    public AlignWithAmpCmd(AprilTagFieldLayout layout, CommandSwerveDrivetrain dt) {
        dt_ = dt ;
        addRequirements(dt_);
        computeDest(layout) ;
    }
    
    private void computeDest(AprilTagFieldLayout layout) {
        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            int tag = -1 ;

            if (alliance.get() == Alliance.Red) {
                tag = 5 ;
            }
            else {
                tag = 6 ;
            }

            Pose2d p = layout.getTagPose(tag).get().toPose2d() ;
            Transform2d t2d = new Transform2d(0.0, -robot_length_ / 2.0, Rotation2d.fromDegrees(0.0)) ;
            p = p.transformBy(t2d) ;
            target_ = new Pose2dWithRotation(p, p.getRotation()) ;
        }
        else {
            has_target_ = false;
        }
    }

    @Override
    public void initialize() {
        dt_.driveTo("auto-amp", null, target_, 1.0, 1.0, 0.0, 0.5, 0.1) ;
    }

    @Override
    public boolean isFinished() {
        return !dt_.isFollowingPath() || !has_target_ ;
    }
}
