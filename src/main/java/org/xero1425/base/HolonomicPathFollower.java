package org.xero1425.base;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.paths.XeroPath;
import org.xero1425.paths.XeroPathSegment;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class HolonomicPathFollower {
    public static class Config {
        public double max_rot_velocity ;
        public double max_rot_acceleration ;
        public double x_p ;
        public double x_i ;
        public double x_d ;
        public double y_p ;
        public double y_i ;
        public double y_d ;
        public double rot_p ;
        public double rot_i ;
        public double rot_d ;
        public double xytolerance ;
        public Rotation2d rot_tolerance ;
        public Supplier<Pose2d> pose_supplier ;
        public Consumer<ChassisSpeeds> output_consumer ;
    }

    private PIDController x_ctrl_ ;
    private PIDController y_ctrl_ ;
    private ProfiledPIDController rot_ctrl_ ;

    private HolonomicDriveController controller_ ;
    private Trajectory traj_ ;
    private boolean driving_ ;
    private double start_time_ ;
    private double timeout_ ;
    private double distance_ ;

    private Pose2dWithRotation start_pose_ ;
    private Pose2dWithRotation end_pose_ ;
    private double rot_pre_ ;
    private double rot_post_ ;

    private Supplier<Pose2d> pose_ ;
    private Consumer<ChassisSpeeds> output_ ;    

    private boolean did_timeout_ ;
    private String path_name_ ;

    private Translation2d last_pos_ ;

    public HolonomicPathFollower(Config cfg) {
        pose_ = cfg.pose_supplier ;
        output_ = cfg.output_consumer ;

        x_ctrl_ = new PIDController(cfg.x_p, cfg.x_i, cfg.x_d) ;
        y_ctrl_ = new PIDController(cfg.y_p, cfg.y_i, cfg.y_d) ;
        TrapezoidProfile.Constraints rot_constraints = new TrapezoidProfile.Constraints(cfg.max_rot_velocity, cfg.max_rot_acceleration) ;
        rot_ctrl_ = new ProfiledPIDController(cfg.rot_p, cfg.rot_i, cfg.rot_d, rot_constraints) ;
        controller_ = new HolonomicDriveController(x_ctrl_, y_ctrl_, rot_ctrl_) ;
        controller_.setTolerance(new Pose2d(cfg.xytolerance, cfg.xytolerance, cfg.rot_tolerance)) ;

        driving_ = false ;
        did_timeout_ = false ;
    }

    //
    // Returns the distance along the current path in meters
    //
    public double getDistance() {
        return distance_ ;
    }

    private Rotation2d getInitialHeading(Translation2d target) {
        double dy = target.getY() - pose_.get().getY() ;
        double dx = target.getX()  - pose_.get().getX() ;
        double angle = Math.atan2(dy, dx) ;
        return Rotation2d.fromRadians(angle) ;
    }

    public void driveTo(String pathname, Pose2d[] imd, Pose2dWithRotation dest, double maxv, double maxa, double pre_rot_time, double pose_rot_time, double to) {
        path_name_ = pathname ;

        Pose2d st = pose_.get() ;
        Rotation2d heading ;
        if (imd != null && imd.length > 0) {
            heading = getInitialHeading(imd[0].getTranslation());
        } else {
            heading = getInitialHeading(dest.getTranslation()) ;
        }
        start_pose_ = new Pose2dWithRotation(st.getTranslation(), heading, st.getRotation()) ;
        start_time_ = Timer.getFPGATimestamp() ;
        end_pose_ = dest ;
        timeout_ = to ;
        distance_ = 0.0 ;

        rot_pre_ = pre_rot_time ;
        rot_post_ = pose_rot_time ;

        TrajectoryConfig config = new TrajectoryConfig(maxv, maxa) ;

        List<Pose2d> pts = new ArrayList<>() ;
        pts.add(start_pose_) ;
        if (imd != null) {
            pts.addAll(Arrays.asList(imd));
        }
        pts.add(dest) ;
        traj_ = TrajectoryGenerator.generateTrajectory(pts, config);

        driving_ = true ;
        last_pos_ = start_pose_.getTranslation() ;
    }

    public void drivePathWithTraj(XeroPath path, double maxv, double maxa, double pre_rot_time, double post_rot_time, double to) {
        XeroPathSegment seg = path.getSegment(0, path.getTrajectoryEntryCount() - 1) ;
        Pose2dWithRotation dest = new Pose2dWithRotation(seg.getX(), seg.getY(), 
                                        Rotation2d.fromDegrees(seg.getHeading()), Rotation2d.fromDegrees(seg.getRotation())) ;

        // The number of points in the intermediate points array
        List<Pose2d> immd = new ArrayList<>() ;

        // The starting point along the generated path for the intermediate points, the current
        // robot pose is the first point
        int index = 5 ;
        while (index < path.getTrajectoryEntryCount()) {
            seg = path.getSegment(0, index) ;
            immd.add(new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading()))) ;
            index += 5 ;
        }

        driveTo(path.getName(), immd.toArray(new Pose2d[0]), dest, maxv, maxa, pre_rot_time, post_rot_time, to) ;
    }

    public boolean didTimeout() {
        return did_timeout_ ;
    }

    public boolean isDriving() {
        return driving_ ;
    }

    public void execute() {
        executeDriveTo() ;
    }

    private void executeDriveTo() {
        if (driving_) {
            Logger.recordOutput("paths:to", path_name_) ;

            double elapsed = Timer.getFPGATimestamp() - start_time_ ;

            Pose2d here = pose_.get() ;
            Trajectory.State st = traj_.sample(elapsed) ;
            Rotation2d rot = rotatationValue(elapsed) ;

            if (st != null && st.poseMeters != null) {
                Logger.recordOutput("paths:target", st.poseMeters) ;
            }

            ChassisSpeeds spd = controller_.calculate(here, st, rot) ;
            output_.accept(spd);

            if (elapsed >= traj_.getTotalTimeSeconds()) {
                if (controller_.atReference()) {
                    driving_ = false ;
                    output_.accept(new ChassisSpeeds()) ;
                }
                else if (elapsed > start_time_ + traj_.getTotalTimeSeconds() + timeout_) {
                    did_timeout_ = true ;
                    driving_ = false ;
                    output_.accept(new ChassisSpeeds()) ;                
                }
            }
            
            distance_ += here.getTranslation().getDistance(last_pos_) ;
            last_pos_ = here.getTranslation() ;
        }
    }

    private Rotation2d rotatationValue(double elapsed) {

        if (elapsed < rot_pre_)
            return start_pose_.getRobotRotation() ;

        if (elapsed > traj_.getTotalTimeSeconds() - rot_post_)
            return end_pose_.getRobotRotation() ;

        //
        // The total time we are rotating
        //
        double span = traj_.getTotalTimeSeconds() - rot_pre_ - rot_post_ ;

        //
        // How far we are along the ramp
        //
        double pcnt = (elapsed - rot_pre_) / span ;

        //
        // Figure out how far along the rotation we should be
        //
        return start_pose_.getRobotRotation().interpolate(end_pose_.getRobotRotation(), pcnt) ;
    }
}
