package frc.robot.subsystems.tracker;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface TrackerIO {
    @AutoLog
    public static class TrackerInputs {
        public int tag_count_ ;
        public Pose2d megatag2_pose_ ;
    }

    public default void updateInputs(TrackerInputs inputs) {
        inputs.tag_count_ = 0 ;
        inputs.megatag2_pose_ = new Pose2d() ;
    }

    public default void setTarget(int tag) {
    }
}
