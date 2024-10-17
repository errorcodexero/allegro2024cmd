package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class NoteVisualizer {

    private static final Transform3d tiltToNote = new Transform3d(
        new Translation3d(Meters.of(0.15), Meters.zero(), Meters.of(0.07)),
        new Rotation3d()
    );
    private static final Transform3d armToNote = new Transform3d(
        new Translation3d(Meters.of(0.23), Meters.zero(), Meters.zero()),
        new Rotation3d(0, Degrees.of(90).in(Radians), 0)
    );

    private static final Pose3d[] emptyPose = new Pose3d[] {};

    // The key for advantagekit to log to
    private final String logkey_;

    // The ComponentVisualizer for the notes to relate their positions to
    private final ComponentVisualizer components_;

    // A supplier of robot pose
    private final Supplier<Pose2d> robotPoseSupplier_;
    
    public NoteVisualizer(String logkey, ComponentVisualizer components, Supplier<Pose2d> robotPoseSupplier) {
        logkey_ = logkey;
        components_ = components;
        robotPoseSupplier_ = robotPoseSupplier;

        updateIntake(false);
        updateArm(true);
    }

    private Pose3d absolutePose(Pose3d robotRelativePose) {
        return new Pose3d(robotPoseSupplier_.get()).transformBy(robotRelativePose.minus(new Pose3d()));
    }

    private Pose3d calculateIntakeNote() {
        return absolutePose(components_.getTiltPose().transformBy(tiltToNote));
    }

    private Pose3d calculateArmNote() {
        return absolutePose(components_.getArmPose().transformBy(armToNote));
    }

    private void publishPose(String subtableName, Pose3d pose, boolean shouldPublish) {
        String subtable = logkey_ + "/" + subtableName;

        if (shouldPublish) {
            Logger.recordOutput(subtable, new Pose3d[] {pose});
        } else {
            Logger.recordOutput(subtable, emptyPose);
        }
    }

    public void updateIntake(boolean hasNote) {
        publishPose("IntakeNote", calculateIntakeNote(), hasNote);
    }

    public void updateArm(boolean hasNote) {
        publishPose("ArmNote", calculateArmNote(), hasNote);
    }
}
