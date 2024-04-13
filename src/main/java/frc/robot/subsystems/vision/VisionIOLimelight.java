package frc.robot.subsystems.vision;

import frc.robot.subsystems.tracker.LimelightHelpers;
import frc.robot.subsystems.tracker.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {
    private String limelight_name_ ;

    public VisionIOLimelight(String name) {
        limelight_name_ = name ;
    }

    public void updateInputs(VisionInputs inputs) {
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_) ;
        inputs.tagCount = pe.tagCount ;
        inputs.pose = pe.pose ;
        inputs.timestampSeconds = pe.timestampSeconds ;
    }
}
