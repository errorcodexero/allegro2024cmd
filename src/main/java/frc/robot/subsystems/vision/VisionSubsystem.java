package frc.robot.subsystems.vision;

import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import frc.robot.subsystems.tracker.LimelightHelpers;
import frc.robot.subsystems.tracker.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends XeroSubsystem {
    private String limelight_name_ ;
    private SwerveDrivetrain db_ ;

    public VisionSubsystem(XeroRobot robot, SwerveDrivetrain db, String name) {
        super(robot, "vision");
    }

    @Override
    public void periodic() {
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_);
        if (pe.tagCount > 0) {
            db_.addVisionMeasurement(pe.pose, pe.timestampSeconds) ;
        }
    }
}
