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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AllegroContainer;
import frc.robot.AprilTags;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.OILed;

public class TrackerSubsystem extends XeroSubsystem {

    private static final String NAME = "tracker" ;

    private SwerveDrivetrain db_ ;
    private boolean has_target_info_ ;
    private int target_number_ ;
    private Pose2d target_pose_ ;
    private TrackerIO io_ ;
    private TrackerInputsAutoLogged inputs_ ;
    private boolean pose_frozen_ ;
    private Pose2d pose_to_use_ ;
    private double last_april_tag_time_ ;               // The time the last april tag (of any kind) was seen
    private Pose2d last_april_tag_pose_ ;               // THe pose from the DRIVEBASE when the last april tag was seen

    private boolean ready_distance_to_target_ ;
    private boolean ready_angle_to_target_ ;
    private boolean ready_time_and_distance_ ;
    private double angle_to_target_ ;
    private double distance_to_target_ ;

    private Trigger ready_for_shoot_trigger_ ;

    public TrackerSubsystem(XeroRobot robot, SwerveDrivetrain db, String name) {
        super(robot, NAME) ;

        db_ = db ;
        target_pose_ = null ;

        io_ = new TrackerIOLimelight(name);
        inputs_ = new TrackerInputsAutoLogged() ;
        pose_frozen_ = false ;
        last_april_tag_time_ = 0 ;

        ready_for_shoot_trigger_ = new Trigger(()-> isOkToShoot()) ;
    }

    public Trigger readyToShoot() {
        return ready_for_shoot_trigger_ ;
    }

    public boolean isOkToShoot() {
        return ready_distance_to_target_ && ready_angle_to_target_ && ready_time_and_distance_ ;
    }

    public boolean isOkToShootAngleDistance() {
        return ready_distance_to_target_ && ready_angle_to_target_ ;
    }

    public void freezePose(boolean v) {
        pose_frozen_ = v ;
        if(v) {
            pose_to_use_ = db_.getState().Pose ;
        }
    }

    public boolean isFrozen() {
        return pose_frozen_ ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

    private Rotation2d getPositionBasedAdj(Rotation2d r) {
        return Rotation2d.fromDegrees(0.0) ;
    }

    private boolean checkTimeAndDistance() {
        boolean ret = true ;

        if (TrackerConstants.kDoAprilTagDistanceCheck) {
            if (last_april_tag_pose_ != null) {
                double dist = last_april_tag_pose_.getTranslation().getDistance(db_.getState().Pose.getTranslation()) ;
                if (dist > TrackerConstants.kAprilTagMaxDistance) {
                    ret = false ;
                }
            }
        }

        if (TrackerConstants.kDoAprilTagTimeCheck) {
            double now = Timer.getFPGATimestamp() ;
            if (now - last_april_tag_time_ > TrackerConstants.kAprilTagMaxTime) {
                ret = false ;
            }
        }

        return ret ;
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

        if (inputs_.tag_count_ > 0) {
            last_april_tag_time_ = Timer.getFPGATimestamp() ;
            last_april_tag_pose_ = db_.getState().Pose ;
        }

        Pose2d robot ;        
        if (pose_frozen_) {
            robot = pose_to_use_ ;
        } else {
            robot = db_.getState().Pose ;
        }

        //
        // We do not see the april tag of interest, use the pose of the robot
        // to aim at the target.
        distance_to_target_ = robot.getTranslation().getDistance(target_pose_.getTranslation()) ;

        //
        // When do we say its ok to shoot just based on pose?
        //
        ready_distance_to_target_ = true; 
        if (distance_to_target_ >= TrackerConstants.kMaximumShotDistance) {
            ready_distance_to_target_ = false;
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

        ready_angle_to_target_ = true;
        if (Math.abs(rheading.getDegrees()) > TrackerConstants.kMaximumShotAngle) {
            ready_angle_to_target_ = false ;
        }

        //
        // Now, find the adjustment based on the original
        //
        Rotation2d rfinal = rheading.plus(getPositionBasedAdj(robot2target)) ;

        //
        // Convert the final rotation to degrees
        //
        angle_to_target_  = rfinal.getDegrees() ;

        ready_time_and_distance_ = checkTimeAndDistance() ;

        if (getVerbose()) {
            Logger.recordOutput("tracker:angle-to-target", angle_to_target_) ;
            Logger.recordOutput("tracker:distance-to-target", distance_to_target_) ;
            Logger.recordOutput("tracker:frozen", pose_frozen_) ;
            Logger.recordOutput("tracker:ready", isOkToShoot()) ;
            Logger.recordOutput("tracker:ready_distance_to_target", ready_distance_to_target_) ;
            Logger.recordOutput("tracker:ready_heading", ready_angle_to_target_) ;
            Logger.recordOutput("tracker:ready_time_and_distance",  ready_time_and_distance_) ;
            Logger.recordOutput("tracker:tagcount", inputs_.tag_count_) ;         
        }

        AllegroContainer container = (AllegroContainer)getRobot().getContainer() ;
        OISubsystem oi = container.getOI() ;
        oi.setLEDState(OILed.TrackerReady, isOkToShoot()) ;  
    }

    public double distance() {
        return distance_to_target_ ;
    }

    public double angle() {
        return angle_to_target_ ;
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
