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

    private static final Measure<Distance> kWheelOffset = Meters.of(0.045); // Robot model wheel offset

    private static final Translation3d kUpdownOrigin = new Translation3d(0.203, 0.0, 0.167 + kWheelOffset.in(Meters)); // Origin of updown gear

    private static final Measure<Distance> kUpdownLength = Inches.of(7.8626771654); // Gear to tilt mount
    private static final Measure<Distance> kElevatorLength = Inches.of(24); // Elevator bottom to arm hinge.

    private static final Measure<Distance> kElevatorHardstopOffset = Inches.of(0.485); // Offset to the hardstop
    private static final Measure<Distance> kClimberOffset = Inches.of(6); // Offset to the bottom of the climber

    private static final Pose3d kElevatorBottom = new Pose3d(
        -0.095636, 0, 0.005998 + kWheelOffset.in(Meters),
        new Rotation3d(0, Degrees.of(-10).in(Radians), 0)
    );

    private final String logkey_;

    private Measure<Angle> updownAngle_; // The angle of the updown
    private Measure<Angle> tiltAngle_; // The angle of the tilt relative to the angle of the updown
    private Measure<Angle> armAngle_; // The angle of the arm

    private Measure<Distance> elevatorHeight_; // How far the elevator has gone up from the bottom.

    private Measure<Distance> climberHeight_; // The height of the climber

    public ComponentVisualizer(String logkey) {
        logkey_ = logkey;

        updownAngle_ = Degrees.of(-123.392);
        tiltAngle_ = Degrees.of(160);
        armAngle_ = Degrees.of(0);
         
        elevatorHeight_ = Meters.of(0);
        climberHeight_ = Meters.of(0);

        update();
    }
    
    public void update() {
        Pose3d updownPose = new Pose3d(
            kUpdownOrigin,
            new Rotation3d(0, updownAngle_.in(Radians), 0)
        );

        Pose3d tiltPose = updownPose.transformBy(
            new Transform3d(
                new Translation3d(kUpdownLength.in(Meters), 0, 0),
                new Rotation3d(0.0, tiltAngle_.in(Radians), 0.0)
            )
        );

        Pose3d elevatorPose = kElevatorBottom.transformBy(
            new Transform3d(
                new Translation3d(0, 0, elevatorHeight_.in(Meters) + kElevatorHardstopOffset.in(Meters)),
                new Rotation3d()
            )
        );

        Pose3d armPose = new Pose3d(
            elevatorPose.transformBy(
                new Transform3d(
                    new Translation3d(0, 0, kElevatorLength.in(Meters)),
                    new Rotation3d()
                )
            ).getTranslation(), new Rotation3d(0, armAngle_.in(Radians), 0)
        );
        
        Pose3d climberPose = kElevatorBottom.transformBy(
            new Transform3d(
                new Translation3d(0, 0, climberHeight_.in(Meters) + kClimberOffset.in(Meters)),
                new Rotation3d()
            )
        );

        Logger.recordOutput(logkey_, updownPose, tiltPose, elevatorPose, armPose, climberPose);
    }

    public void setUpdownAngle(Measure<Angle> updownAngle) {
        updownAngle_ = updownAngle;
        update();
    }

    public void setTiltAngle(Measure<Angle> tiltAngle) {
        tiltAngle_ = tiltAngle;
        update();
    }

    public void setElevatorHeight(Measure<Distance> elevatorHeight) {
        elevatorHeight_ = elevatorHeight;
        update();
    }

    public void setArmAngle(Measure<Angle> armAngle) {
        armAngle_ = armAngle;
        update();
    }

    public void setClimberHeight(Measure<Distance> climberHeight) {
        climberHeight_ = climberHeight;
        update();
    }

}