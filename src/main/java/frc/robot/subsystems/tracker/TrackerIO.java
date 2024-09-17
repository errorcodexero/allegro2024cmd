package frc.robot.subsystems.tracker;

import org.littletonrobotics.junction.AutoLog;

public interface TrackerIO {
    @AutoLog
    public static class TrackerInputs {
        public int tag_count_ ;
    }

    public default void updateInputs(TrackerInputs inputs) {
        inputs.tag_count_ = 0 ;
    }

    public default void setTarget(int tag) {
    }
}
