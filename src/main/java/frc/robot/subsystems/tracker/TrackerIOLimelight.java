package frc.robot.subsystems.tracker;

import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.base.LimelightHelpers.PoseEstimate;

public class TrackerIOLimelight implements TrackerIO {
    private String limelight_name_ ;

    public TrackerIOLimelight(String name) {
        limelight_name_ = name ;
    }

    public void updateInputs(TrackerIO.TrackerInputs inputs) {
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_) ;

        inputs.tag_count_ = est != null ? est.tagCount : 0 ;
        inputs.x_ = (est != null) ? est.pose.getX() : Double.NaN ;
        inputs.y_ = (est != null) ? est.pose.getY() : Double.NaN ;
        inputs.heading_ = (est != null) ? est.pose.getRotation().getDegrees() : Double.NaN ;
        inputs.tags_ = "" ;

        LimelightResults res = LimelightHelpers.getLatestResults(limelight_name_) ;
        if (res != null) {
            for(LimelightHelpers.LimelightTarget_Fiducial fud : res.targets_Fiducials) {
                if (fud != null) {
                    if (inputs.tags_.length() > 0)
                        inputs.tags_ += "," ;
                    inputs.tags_ += Double.toString(fud.fiducialID) + " " ;
                }
            }
        }
    }
}
