package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.math.XeroMath;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class AutoTrapCommand extends Command {

    //
    // April Tags
    //
    // Red: 11, 12, 13
    // Blue: 14, 15, 16
    //

    private enum State {
        Starting,
        LookForAprilTag,
        DriveToTrap,
        Wait,
        Trapping,
        Done
    }

    private static double kExtraSpacing1 = 1.5 ;
    private static double kExtraSpacing2 = 0.0 ;    
    private static double kLeftSpacing1 = -0.1 ;
    private static double kLeftSpacing2 = -0.1;
    private static double kMaxDistance = 4.0 ;
    private static int kSimulatedTag = 11 ;

    private String limelight_name_ ;

    private OISubsystem oi_ ;
    private TrampSubsystem tramp_ ;
    private CommandSwerveDrivetrain db_ ;

    private Command trap_command_ ;
    private AprilTagFieldLayout layout_ ;
    private State state_ ;
    private int target_tag_ ;
    private int[] desired_tags_ = new int[3] ;

    public AutoTrapCommand(String name, AprilTagFieldLayout layout, OISubsystem oi, TrampSubsystem tramp, CommandSwerveDrivetrain dt) {
        limelight_name_ = name ;
        db_ = dt ;
        oi_ = oi ;
        tramp_ = tramp ;
        layout_ = layout ;
        addRequirements(db_) ;
        state_ = State.Starting ;

        computeTarget(11) ;
    }

    private Pose2dWithRotation[] computeTarget(int tag) {
        Pose2dWithRotation [] ret = new Pose2dWithRotation[2] ;
        Pose2d pt ;
        Rotation2d ptrt ;

        Pose2d tagpose = layout_.getTagPose(tag).get().toPose2d() ;

        pt = computeProjectedTrapPoint(tagpose, kExtraSpacing1, kLeftSpacing1);
        ptrt = pt.getRotation() ;
        ret[0] = new Pose2dWithRotation(pt.getTranslation(), ptrt, ptrt) ;

        pt = computeProjectedTrapPoint(tagpose, kExtraSpacing2, kLeftSpacing2) ;
        ptrt = pt.getRotation() ;
        ret[1] = new Pose2dWithRotation(pt.getTranslation(), ptrt, ptrt) ;

        double dist = ret[0].getTranslation().getDistance(db_.getState().Pose.getTranslation()) ;
        if (dist > kMaxDistance) {
            ret = null ;
        }

        return ret;
    }

    private Pose2d projectedAlongHeading(Pose2d p, double projection) {
        double dx = Math.cos(p.getRotation().getRadians()) * projection ;
        double dy = Math.sin(p.getRotation().getRadians()) * projection ;
        return new Pose2d(p.getX() + dx, p.getY() + dy, p.getRotation()) ;
    }  

    private Pose2d computeProjectedTrapPoint(Pose2d tag, double xspacing, double yspacing) {
        // Find the point projected along the heading given by the tag
        Pose2d ret1 = projectedAlongHeading(tag, xspacing) ;

        // Find a point at the end of the projection, rotated by 90 degrees
        Pose2d pt = new Pose2d(ret1.getTranslation(), ret1.getRotation().rotateBy(Rotation2d.fromDegrees(90.0))) ;

        // Find a point at the end of the offset at the end of the projection
        Pose2d ret2 = projectedAlongHeading(pt, yspacing) ;

        // Take the original heading witht he final point
        Pose2d ret = new Pose2d(ret2.getTranslation(), tag.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))) ;

        return ret ;
    }

    @Override
    public void initialize() {
        target_tag_ = -1 ;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            desired_tags_[0] = -1 ;
            desired_tags_[1] = -1 ; 
            desired_tags_[2] = -1 ;
        }
        else if (alliance.get() == Alliance.Red) {
            desired_tags_[0] = 11 ;
            desired_tags_[1] = 12 ;
            desired_tags_[2] = 13 ;
        }
        else {
            desired_tags_[0] = 14 ;
            desired_tags_[1] = 15 ;
            desired_tags_[2] = 16 ;
        }
        state_ = State.LookForAprilTag ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case Starting:
                break ;
            case LookForAprilTag:
                if (oi_.isAbortPressed()) {
                    db_.stopPath() ;
                    state_ = State.Done ;
                }
                else {
                    lookForAprilTag() ;
                }
                break; 

            case DriveToTrap:
                if (oi_.isAbortPressed()) {
                    db_.stopPath() ;
                    state_ = State.Done ;
                }
                else if (!db_.isFollowingPath()) {
                    state_ = State.Wait ;
                }
                break ;

            case Wait:
                if (oi_.isAbortPressed()) {
                    db_.stopPath() ;
                    state_ = State.Done ;
                }
                else if (oi_.isAutoTrapPressed()) {
                    trap_command_ = tramp_.trapCommand() ;
                    CommandScheduler.getInstance().schedule(trap_command_) ;                    
                    state_ = State.Trapping ;
                }
                break ;

            case Trapping:
                if (trap_command_.isFinished()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("auto-trap:tag", Integer.toString(target_tag_)) ;
        Logger.recordOutput("auto-trap:state", state_) ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
    
    @Override
    public void end(boolean interrupted) {
        db_.stopPath() ;
    }

    private void lookForAprilTag() {
        if (XeroRobot.isReal()) {
            //
            // Find the april tag based on what we see
            //
            LimelightResults results = LimelightHelpers.getLatestResults(limelight_name_) ;
            target_tag_ = -1 ;

            for(int i = 0 ; i < results.targets_Fiducials.length ; i++) {
                for(int j = 0 ; j < desired_tags_.length ; j++) {
                    if (results.targets_Fiducials[i].fiducialID == desired_tags_[j]) {
                        target_tag_ = desired_tags_[j] ;
                        break ;
                    }
                }
            }
        }
        else {
            //
            // Force the april tag based on the simulation
            //
            target_tag_ = kSimulatedTag ;
        }

        if (target_tag_ != -1) {
            Pose2dWithRotation [] waypoints = computeTarget(target_tag_) ;
            if (waypoints != null) {
                Pose2d immds[] = new Pose2d[] { waypoints[0] } ;
                db_.driveTo("auto-amp", immds, waypoints[1], 1.0, 1.0, 0.0, 1.0, 1.0) ;
                state_ = State.DriveToTrap ;
            }
            else {
                state_ = State.Done ;
            }
        }
        else {
            state_ = State.Done ;
        }
    }

  
}
