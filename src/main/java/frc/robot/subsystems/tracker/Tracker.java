package frc.robot.subsystems.tracker;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AprilTags;

public class Tracker extends XeroSubsystem {
    private SwerveDrivetrain db_ ;
    private boolean has_target_info_ ;
    private int target_number_ ;
    private Pose2d target_pose_ ;
    private TrackerIO io_ ;
    private TrackerInputsAutoLogged inputs_ ;

    private boolean ok_to_shoot_ ;
    private double angle_to_target_ ;
    private double distance_to_target_ ;

    public Tracker(XeroRobot robot, SwerveDrivetrain db, String name) {
        super(robot, "tracker") ;

        db_ = db ;
        target_pose_ = null ;

        ok_to_shoot_ = false ;

        io_ = new TrackerIOLimelight(name);
        inputs_ = new TrackerInputsAutoLogged() ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

    private Rotation2d getPositionBasedAdj(Rotation2d r) {
        return Rotation2d.fromDegrees(0.0) ;
    }

    @Override
    public void periodic() {
        periodicStart();

        if (!has_target_info_) {
            has_target_info_ = getTargetPose() ;
            if (!has_target_info_) {
                //
                // We have no target, so we can't do anything
                //
                return ;
            }
        }

        io_.updateInputs(inputs_) ;
        Logger.processInputs("tracker", inputs_);

        Pose2d robot = db_.getState().Pose ;

        //
        // We do not see the april tag of interest, use the pose of the robot
        // to aim at the target.
        ok_to_shoot_ = false ;
        distance_to_target_ = robot.getTranslation().getDistance(target_pose_.getTranslation()) ;

        //
        // When do we say its ok to shoot just based on pose?
        //
        if (distance_to_target_ < TrackerConstants.kOkToShootPoseDistance) {
            ok_to_shoot_ = true ;
        }

        Translation2d diff = target_pose_.getTranslation().minus(robot.getTranslation()) ;

        //
        // This is the angle from the center of the robot to the center of the target
        //
        Rotation2d robot2target = new Rotation2d(diff.getX(), diff.getY()) ;

        //
        // Since we shoot out the back, we need to rotate the angle by 180 degrees
        //
        Rotation2d rheading = robot2target.plus(Rotation2d.fromDegrees(180.0)) ;

        //
        // Now, find the adjustment based on the original
        //
        Rotation2d rfinal = rheading.plus(getPositionBasedAdj(robot2target)) ;

        //
        //
        //
        angle_to_target_  = rfinal.getDegrees() ;

        if (getVerbose()) {
            Logger.recordOutput("tracker:angle-to-target", angle_to_target_) ;
            Logger.recordOutput("tracker:distance-to-target", distance_to_target_) ;
        }

        periodicEnd();
    }

    public double distance() {
        return distance_to_target_ ;
    }

    public double angle() {
        return angle_to_target_ ;
    }

    public boolean okToShoot() {
        return ok_to_shoot_ ;
    }

    private boolean getTargetPose() {
        Optional<Alliance> alliance = DriverStation.getAlliance() ;
        if (alliance.isEmpty())
            return false ;

        if (alliance.get() == Alliance.Red) {
            target_number_ = AprilTags.RED_SPEAKER_CENTER ;
        }
        else {
            target_number_ = AprilTags.BLUE_SPEAKER_CENTER ;
        }

        Optional<Pose3d> pose = getRobot().getFieldLayout().getTagPose(target_number_) ;
        if (pose.isEmpty()) {
            return false ;
        }

        target_pose_ = pose.get().toPose2d() ;

        //
        // This will cause TX, TY, TA, and TV to be about the target_number_
        // april tag.
        //
        io_.setTarget(target_number_) ;

        return true ;
    }

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }
}
