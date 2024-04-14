package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionInputs {
        public int tagCount = 0 ;
        public Pose2d pose = new Pose2d() ;
        public double timestampSeconds = 0.0 ;
    }

    public default void updateInputs(VisionInputs inputs) {
    }
}
