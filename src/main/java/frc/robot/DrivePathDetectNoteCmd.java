package frc.robot;

import java.util.ArrayList;

import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.base.LimelightHelpers.LimelightTarget_Detector;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivePathDetectNoteCmd extends Command {
    private String llname_ ;
    private CommandSwerveDrivetrain dt_ ;
    private Pose2dWithRotation pts_[] ;
    private double maxv_ ;
    private double maxa_ ;
    private double trans_dist_ ;
    private boolean done_ ;
    private boolean tracking_note_ ;
    private Translation2d notepos_ ;
    private PIDController x_ ;
    private PIDController y_ ;
    private PIDController theta_ ;
    private LimelightTarget_Detector note_ ;

    public DrivePathDetectNoteCmd(String llname, CommandSwerveDrivetrain dt, Pose2dWithRotation pts[], 
                                    Translation2d notepos, double maxv, double maxa, double transdist) {
        dt_ = dt ;
        pts_ = pts ;
        llname_ = llname ;
        notepos_ = notepos ;
        maxv_ = maxv ;
        maxa_ = maxa ;
        trans_dist_ = transdist ;

        x_ = new PIDController(0.0, 0.0, 0.0) ;
        y_ = new PIDController(0.0, 0.0, 0.0) ;
        theta_ = new PIDController(0.0, 0.0, 0.0) ;
    }

    @Override
    public void initialize() {
        Pose2d[] imd = null ;

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
        findClosest();

        if (!tracking_note_ && !dt_.isFollowingPath()) {
            done_ = true ;
        }
        else if (!tracking_note_) {
            double dist = notepos_.getDistance(dt_.getState().Pose.getTranslation()) ;
            if (dist < trans_dist_ && note_ != null) {
                dt_.stopPath(false) ;
                tracking_note_ = true ;
                ChassisSpeeds spd = getTrackingChassisSpeeds() ;
                dt_.setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ; 
            }
        }
        else {
            ChassisSpeeds spd = getTrackingChassisSpeeds() ;
            dt_.setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ; 
        }
    }

    private ChassisSpeeds getTrackingChassisSpeeds() {
        double dist = notepos_.getDistance(dt_.getState().Pose.getTranslation()) ;

        double xRobotRel = x_.calculate(dist, 0.0) ;
        double yRobotRel = y_.calculate(note_.tx, 0.0) ;
        double thetaRobotRel = theta_.calculate(dt_.getState().Pose.getRotation().getDegrees(), 0.0) ;

        return new ChassisSpeeds(xRobotRel, yRobotRel, thetaRobotRel);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }

    private double mapAngle(double angle) {
        angle = (int)(angle / 5.0) * 5.0 ;
        return angle ;
    }   

    private void findClosest() {
        LimelightTarget_Detector [] notes = LimelightHelpers.getLatestResults(llname_).targets_Detector ;

        double minseen = 0.0 ;
        ArrayList<Integer> possible = new ArrayList<>() ;
        ArrayList<Double> yangles = new ArrayList<>() ;

        for(int i = 0 ; i < notes.length ; i++) {
            double maxang = mapAngle(notes[i].tx) ;
            if (maxang > minseen)
                continue ;

            if (maxang == minseen) {
                possible.add(i) ;
                yangles.add(notes[i].ty) ;
            } else {
                minseen = maxang ;
                possible.clear() ;
                yangles.clear() ;

                possible.add(i) ;
                yangles.add(notes[i].ty) ;
            }
        }

        int ret = -1 ;
        if (possible.size() > 1) {
            double miny = 1000.0 ;
            for(int i = 0 ; i < yangles.size() ; i++) {
                if (Math.abs(yangles.get(i)) < miny) {
                    miny = Math.abs(yangles.get(i)) ;
                    ret = i ;
                }
            }
        }
        else if (possible.size() == 1) {
            ret = possible.get(0) ;
        }

        if (ret == -1) {
            note_ = null ;
        }
        else {
            note_ = notes[ret] ;
        }
    }
}
