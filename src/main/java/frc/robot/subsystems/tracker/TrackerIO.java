package frc.robot.subsystems.tracker;

import org.littletonrobotics.junction.AutoLog;

public interface TrackerIO {
    @AutoLog
    public static class TrackerInputs {
        public int tag_count_ ;
        public double x_ ;
        public double y_ ;
        public double heading_ ;
    }

    public default void updateInputs(TrackerInputs inputs) {
        inputs.tag_count_ = 0 ;
        inputs.x_ = Double.NaN ;
        inputs.y_ = Double.NaN ;
        inputs.heading_ = Double.NaN ;
    }

    public default void setTarget(int tag) {
    }
}
