package frc.robot.subsystems.tracker;

import frc.robot.LimelightHelpers;

public class TrackerIOLimelight implements TrackerIO {
    private String limelight_name_ ;

    public TrackerIOLimelight(String name) {
        limelight_name_ = name ;
    }

    public void updateInputs(TrackerIO.TrackerInputs inputs) {
        inputs.tv = LimelightHelpers.getTV(limelight_name_) ;
        inputs.tx = LimelightHelpers.getTX(limelight_name_) ;
        inputs.ty = LimelightHelpers.getTY(limelight_name_) ;
    }

    public void setTarget(int tag) {
        LimelightHelpers.setPriorityTagID(limelight_name_, tag) ;
    }
}
