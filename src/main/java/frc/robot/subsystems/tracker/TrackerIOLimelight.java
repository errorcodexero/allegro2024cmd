package frc.robot.subsystems.tracker;

import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;

public class TrackerIOLimelight implements TrackerIO {
    private String limelight_name_ ;

    public TrackerIOLimelight(String name) {
        limelight_name_ = name ;
    }

    public void updateInputs(TrackerIO.TrackerInputs inputs) {
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_) ;

        inputs.tag_count_ = est != null ? est.tagCount : 0 ;
        inputs.megatag2_pose_ = est != null ? est.pose : new Pose2d() ;
    }
}
