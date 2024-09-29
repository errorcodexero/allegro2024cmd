package frc.robot.subsystems.tracker;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class TrackerIOLimelight implements TrackerIO {
    private String limelight_name_ ;

    public TrackerIOLimelight(String name) {
        limelight_name_ = name ;
    }

    public void updateInputs(TrackerIO.TrackerInputs inputs) {
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_) ;
        inputs.tag_count_ = est != null ? est.tagCount : 0 ;
    }
}
