package frc.robot.subsystems.tracker;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;
import org.xero1425.XeroMath;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTags;

public class Tracker extends XeroSubsystem {
    private SwerveDrivetrain db_ ;
    private boolean has_target_info_ ;
    private int target_number_ ;
    private Pose2d target_pose_ ;
    private TrackerIO io_ ;
    private TrackerInputsAutoLogged inputs_ ;

    private String source_ ;
    private int zone_ ;
    private double angle_offset_ ;
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

    @Override
    public void periodic() {
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

        if (inputs_.tv) {
            //
            // We see the april tag of interest, use the TX and TY values to
            // aim at the target.
            //
            ok_to_shoot_ = true ;
            angle_to_target_ = -inputs_.tx + angle_offset_ ;
            distance_to_target_ = (TrackerConstants.kTargetHeight - TrackerConstants.kCameraHeight) / Math.tan(Math.toRadians(TrackerConstants.kCameraAngle + inputs_.ty)) ;
            source_ = "AprilTag" ;

        } else {
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

            Translation2d diff = robot.getTranslation().minus(target_pose_.getTranslation());
            Rotation2d rot = new Rotation2d(diff.getX(), diff.getY()) ;
            angle_to_target_  = XeroMath.normalizeAngleDegrees(rot.getDegrees() - robot.getRotation().getDegrees()) + angle_offset_ ;

            source_ = "RobotPose" ;
        }

        Logger.recordOutput("tracker-ok-to-shoot", ok_to_shoot_) ;
        Logger.recordOutput("tracker-angle-to-target", angle_to_target_) ;
        Logger.recordOutput("tracker-distance-to-target", distance_to_target_) ;
        Logger.recordOutput("tracker-angle-offset", angle_offset_) ;
        Logger.recordOutput("tracker-source", source_) ;
        Logger.recordOutput("tracker-zone", zone_) ;
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

    public Command setOffsetCommand() {
        return runOnce(this::setOffset) ;
    }

    public Command clearOffsetCommand() {
        return runOnce(() -> angle_offset_ = 0.0) ;
    }

    private double getTargetAngle() {
        double ret = 0.0 ;

        if (target_pose_ != null) {
            Pose2d robot = db_.getState().Pose ;

            if (target_number_ == AprilTags.RED_SPEAKER_CENTER) {
                double angle = Math.atan2(target_pose_.getY() - robot.getY(), target_pose_.getX() - robot.getX()) ;
                ret = XeroMath.normalizeAngleDegrees(Math.toDegrees(angle)) ;                
            }
            else {
                double angle = Math.atan2(target_pose_.getY() - robot.getY(), target_pose_.getX() - robot.getX()) ;
                ret = XeroMath.normalizeAngleDegrees(Math.toDegrees(angle) + 180) ;
            }
        }
        return ret ;
    }    

    private void setOffset() {
        double effective = getTargetAngle() ;

        if (target_number_ == AprilTags.BLUE_SPEAKER_CENTER) {
            if (effective <= 20 && effective >= -20) {
                zone_ = 1 ;
                angle_offset_ = 0 ;
            }
            else if (effective < -20 && effective >= -40) {
                zone_ = 2 ;
                angle_offset_ = -5.0 ;
            }
            else if (effective < -40 && effective >= -70) {
                zone_ = 3 ;
                angle_offset_ = -7.5 ;
            }
            else if (effective > 20 && effective <= 40) {
                zone_ = 4 ;
                angle_offset_ = 5.0 ;
            }
            else if (effective > 40 && effective <= 70) { 
                zone_ = 5 ;
                angle_offset_ = 7.5 ;
            } 
            else {
                zone_ = 6 ;
                angle_offset_ = -7.5 ;
            }
        }
        else {
            if (effective <= 20 && effective >= -20) {
                zone_ = 1 ;
                angle_offset_ = 0 ;
            }
            else if (effective < -20 && effective >= -40) {
                zone_ = 2 ;
                angle_offset_ = -15 ;
            }
            else if (effective < -40 && effective >= -70) {
                zone_ = 3 ;
                angle_offset_ = -7.5 ;
            }
            else if (effective > 20 && effective <= 40) {
                zone_ = 4 ;
                angle_offset_ = 0.0 ;
            }
            else if (effective > 40 && effective <= 70) { 
                zone_ = 5 ;
                angle_offset_ = 0.0 ;
            } 
            else {
                zone_ = 6 ;
                angle_offset_ = -7.5 ;
            }            
        }
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
}
