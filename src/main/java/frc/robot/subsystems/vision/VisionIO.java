package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionInputs {
        public int tagCount ;
        public Pose2d pose ;
        public double timestampSeconds ;
    }

    public default void updateInputs(VisionInputs inputs) {
    }
}
