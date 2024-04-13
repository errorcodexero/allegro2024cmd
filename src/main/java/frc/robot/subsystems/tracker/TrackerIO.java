package frc.robot.subsystems.tracker;

import org.littletonrobotics.junction.AutoLog;

public interface TrackerIO {
    @AutoLog
    public static class TrackerInputs {
        public boolean tv = false ;
        public double ty = 0.0 ;
        public double tx = 0.0 ;
    }

    public default void updateInputs(TrackerInputs inputs) {
    }

    public default void setTarget(int tag) {
    }
}
