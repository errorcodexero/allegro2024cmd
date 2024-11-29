package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gamepiecetracker.GamePieceTracker;

public class DrivePathDetectNoteCmd extends Command {
    private CommandSwerveDrivetrain dt_ ;
    private Pose2dWithRotation pts_[] ;
    private double maxv_ ;
    private double maxa_ ;
    private double trans_dist_ ;
    private boolean done_ ;
    private boolean tracking_note_ ;
    private PIDController x_ ;
    private PIDController y_ ;
    private PIDController theta_ ;
    private GamePieceTracker tracker_ ;
    private ChassisSpeeds tracking_speeds_ ;
    private int lost_gp_count_ ;
    private double max_x_vel_ ;
    private double max_y_vel_ ;
    private double max_theta_vel_ ;

    public DrivePathDetectNoteCmd(String llname, GamePieceTracker tracker, CommandSwerveDrivetrain dt, Pose2dWithRotation pts[], 
                                    Translation2d notepos, double maxv, double maxa, double transdist) {
        dt_ = dt ;
        pts_ = pts ;
        maxv_ = maxv ;
        maxa_ = maxa ;
        trans_dist_ = transdist ;
        tracker_ = tracker ;

        x_ = new PIDController(3.0, 0.0, 0.0) ;
        y_ = new PIDController(1.0, 0.0, 0.0) ;
        theta_ = new PIDController(0.075, 0.0, 0.0) ;
        theta_.enableContinuousInput(-180.0, 180.0);

        max_x_vel_ = 4.0 ;
        max_y_vel_ = 0.0 ;
        max_theta_vel_ = Math.toRadians(180.0) ;
    }

    @Override
    public void initialize() {
        Pose2d[] imd = null ;

        lost_gp_count_ = 0 ;

        if (pts_.length > 2) {
            imd = new Pose2d[pts_.length - 2] ;
            int i = 1 ;
            while (i < pts_.length - 1) {
                imd[i - 1] = pts_[i] ;
                i++ ;
            }
        }

        done_ = false ;
        tracking_note_ = false ;

        dt_.seedFieldRelative() ;
        dt_.driveTo("test", imd, pts_[pts_.length - 1], maxv_, maxa_, 0.0, 0.0, 1.0) ;
    }

    @Override
    public void execute() {

        String mode = "NONE" ;

        if (!tracking_note_) {
            if (!dt_.isFollowingPath()) {
                //
                // We are following the path and the path is complete.
                //
                mode = "done" ;
                done_ = true ;
            }
            else {
                //
                // We are following the path
                //
                mode = "path" ;
                if (tracker_.getDistanceClosestGamePiece() < trans_dist_) {
                    dt_.stopPath(false) ;
                    tracking_note_ = true ;

                    ChassisSpeeds spd = getTrackingChassisSpeeds() ;
                    dt_.setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ; 
                }
            }
        }
        else {
            mode = "note" ;
            ChassisSpeeds spd = getTrackingChassisSpeeds() ;
            dt_.setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ; 

            if (done_) {
                mode = "done" ;
            }
        }

        Logger.recordOutput("gptracker:mode", mode) ;
    }

    private ChassisSpeeds getTrackingChassisSpeeds() {
        if (tracker_.seesGamePieces()) {
            double dist = tracker_.getDistanceClosestGamePiece() ;
            double angle = tracker_.getAngleClosestGamePiece() ;

            Logger.recordOutput("gptracker:sangle", angle) ;

            double xvel = x_.calculate(0.0, dist) ; 
            if (Math.abs(xvel) > max_x_vel_)
                xvel = max_x_vel_ * Math.signum(xvel) ;

            double yvel = y_.calculate(angle, 0.0) ;
            if (Math.abs(yvel) > max_y_vel_)
                yvel = max_y_vel_ * Math.signum(yvel) ;

            double thetavel = theta_.calculate(angle, 0.0) ;
            if (Math.abs(thetavel) > max_theta_vel_)
                thetavel = max_theta_vel_ * Math.signum(thetavel) ;

            tracking_speeds_ = new ChassisSpeeds(xvel, yvel, thetavel) ;

            lost_gp_count_ = 0 ;
        }
        else {
            lost_gp_count_++ ;
            if (lost_gp_count_ > 25) {
                tracking_speeds_ = new ChassisSpeeds() ;
                done_ = true ;
            }
        }

        Logger.recordOutput("gptracker:xvel", tracking_speeds_.vxMetersPerSecond) ;  
        Logger.recordOutput("gptracker:yvel", tracking_speeds_.vyMetersPerSecond) ;  
        Logger.recordOutput("gptracker:avel", tracking_speeds_.omegaRadiansPerSecond) ;  
        Logger.recordOutput("gptracker:lost", lost_gp_count_) ;  

        return tracking_speeds_ ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
