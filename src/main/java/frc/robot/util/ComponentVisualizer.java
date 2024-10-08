package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class ComponentVisualizer {

    // The height of the robot is offset by the height of the wheels being added in the model.
    // So all absolute coordinates are offset by this value, and it must be added to Z.
    private static final Measure<Distance> kWheelOffset = Meters.of(0.045);

    // The rotation mate point of the updown.
    private static final Translation3d kUpdownOrigin = new Translation3d(0.203, 0.0, 0.167 + kWheelOffset.in(Meters));

    // The length between the rotation point of the updown to the rotation point of the tilt.
    private static final Measure<Distance> kUpdownLength = Inches.of(7.8626771654);

    // The length between the bottom of the elevator to the arm hinge.
    private static final Measure<Distance> kElevatorLength = Inches.of(24);

    // The bottom of the elevator not including the hard stop, so we need to add the width of the hardstop in our calculations.
    private static final Pose3d kElevatorBottom = new Pose3d(
        -0.095636, 0, 0.005998 + kWheelOffset.in(Meters),
        new Rotation3d(0, Degrees.of(-10).in(Radians), 0)
    );
    
    // The elevator hardstop width
    private static final Measure<Distance> kElevatorHardstopOffset = Inches.of(0.485);
    
    // The offset in height from the bottom of the elevator hardstop to the hole in the climber hooks.
    private static final Measure<Distance> kClimberOffset = Inches.of(6);
    
    // The key for advantagekit to log to
    private final String logkey_;

    // Initialize poses at robot origin until updated.
    private Pose3d updownPose_ = new Pose3d();
    private Pose3d tiltPose_ = new Pose3d();
    private Pose3d elevatorPose_ = new Pose3d();
    private Pose3d armPose_ = new Pose3d();
    private Pose3d climberPose_ = new Pose3d();
    
    public ComponentVisualizer(String logkey) {
        logkey_ = logkey;
    }

    private void publish() {
        Logger.recordOutput(logkey_, updownPose_, tiltPose_, elevatorPose_, armPose_, climberPose_);
    }

    // Updates are grouped based on dependance.
    // Ex. the tilts pose depends on the updowns pose, so we update them together.

    /**
     * Calculates and publishes 3d poses for the intake shooter.
     * @param updownAngle The angle of the updown from facing directly forward.
     * @param tiltAngle The angle of the tilt relative to perpendicular from the updown.
     */
    public void updateIntakeShooter(Measure<Angle> updownAngle, Measure<Angle> tiltAngle) {
        updownPose_ = new Pose3d(
            kUpdownOrigin,
            new Rotation3d(0, updownAngle.in(Radians), 0)
        );

        tiltPose_ = updownPose_.transformBy(
            new Transform3d(
                new Translation3d(kUpdownLength.in(Meters), 0, 0),
                new Rotation3d(0.0, tiltAngle.plus(Degrees.of(90)).in(Radians), 0.0)
            )
        );

        publish();
    }

    /**
     * Calculates and publishes 3d poses for the tramp.
     * @param elevatorHeight The distance the elevator has travelled from the hardstop.
     * @param armAngle The angle of the arm relative to its rest position.
     */
    public void updateTramp(Measure<Distance> elevatorHeight, Measure<Angle> armAngle) {
        elevatorPose_ = kElevatorBottom.transformBy(
            new Transform3d(
                new Translation3d(0, 0, elevatorHeight.in(Meters) + kElevatorHardstopOffset.in(Meters)),
                new Rotation3d()
            )
        );

        armPose_ = new Pose3d(
            elevatorPose_.transformBy(
                new Transform3d(
                    new Translation3d(0, 0, kElevatorLength.in(Meters)),
                    new Rotation3d()
                )
            ).getTranslation(), new Rotation3d(0, armAngle.plus(Degrees.of(80)).in(Radians), 0)
        );

        publish();
    }
    
    /**
     * Updates and publishes the 3d pose of the climber hooks.
     * @param climberHeight The height the climbers have traveled from their reset position.
     */
    public void updateClimber(Measure<Distance> climberHeight) {
        climberPose_ = kElevatorBottom.transformBy(
            new Transform3d(
                new Translation3d(0, 0, climberHeight.in(Meters) + kClimberOffset.in(Meters)),
                new Rotation3d()
            )
        );

        publish();
    }

}