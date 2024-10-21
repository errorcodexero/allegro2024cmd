package frc.robot.commands.trapcmds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroTimer;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.SwerveRotateToAngle;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

//
// This version of auto trap looks for an april tag and then creates a path to a first point
// which is kExtraSpacing1 away from the tag.  It then rotates the robot to face the tag and
// then drives to a second point which is kExtraSpacing2 away from the tag.  The robot then
// waits for the driver to press the auto trap button to start the trapping process.  This approach
// keeps the amount of path following to a minimum after the big rotation.
//

public class AutoTrap2Command extends AutoTrapBase {

    private static double kExtra2Spacing1 = 1.5 ;       // Off from the april tag toward the robot
    private static double kExtra2Spacing2 = 0.165;      // Off from the april tag toward the robot
    private static double kRight2Spacing1 = 0.0 ;       // Positive moves to the right
    private static double kRight2Spacing2 = 0.0 ;       // Positive moves to the right    

    private enum State {
        Starting,
        LookForAprilTag,
        Delay1,
        DriveToTrap1,
        Rotate,
        Delay2,
        DriveToTrap2,
        Wait,
        Trapping,
        Done
    }

    private static double kMaxDistance = 3.5 ;
    private static double kMinDistance = 2.0 ;
    private static int kSimulatedTag = 11 ;

    private OISubsystem oi_ ;
    private TrampSubsystem tramp_ ;
    private CommandSwerveDrivetrain db_ ;

    private Command trap_command_ ;
    private State state_ ;
    private int target_tag_ ;
    private int[] desired_tags_ = new int[3] ;
    private Pose2dWithRotation [] waypoints_ = null ;
    private SwerveRotateToAngle rotate_ = null ;
    private XeroTimer delay1_timer_ ;
    private XeroTimer delay2_timer_ ;

    public AutoTrap2Command(String limelight, AprilTagFieldLayout layout, OISubsystem oi, TrampSubsystem tramp, CommandSwerveDrivetrain dt) {
        super(dt, limelight, layout, kMaxDistance, kMinDistance) ;
        db_ = dt ;
        oi_ = oi ;
        tramp_ = tramp ;
        addRequirements(db_) ;
        state_ = State.Starting ;
        delay2_timer_ = new XeroTimer("delay2-timer", 1.0) ;
        delay1_timer_ = new XeroTimer("delay1-timer", 1.0) ;
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

            case Delay1:
                if (delay1_timer_.isExpired()) {
                    db_.setMegaTag2(true) ;

                    Rotation2d angle = waypoints_[0].getRobotRotation().rotateBy(Rotation2d.fromDegrees(180.0)) ;
                    rotate_ = new SwerveRotateToAngle(db_, angle, 1.0, 1.0) ;
                    state_ = State.Rotate ;
                    CommandScheduler.getInstance().schedule(rotate_);
                }
                break ;

            case DriveToTrap1:
                if (oi_.isAbortPressed()) {
                    db_.stopPath() ;
                    state_ = State.Done ;
                }
                else {
                    driveToTrap1State() ;
                }
                break ;

            case Rotate:
                if (oi_.isAbortPressed()) {
                    rotate_.cancel();
                    state_ = State.Done ;
                }
                else {
                    rotateState() ;
                }            
                break; 

            case Delay2:
                if (oi_.isAbortPressed()) {
                    state_ = State.Done ;
                }
                else if (delay2_timer_.isExpired()) {
                    db_.driveTo("auto-trap2", null, waypoints_[1], 1.0, 1.0, 0.0, 0.0, 1.0) ;
                    state_ = State.DriveToTrap2 ;
                }
                break ;

            case DriveToTrap2:
                if (oi_.isAbortPressed()) {
                    db_.stopPath();
                    state_ = State.Done ;
                }
                else {
                    driveToTrap2State() ;
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

        Logger.recordOutput("autotrap2:tag", Integer.toString(target_tag_)) ;
        Logger.recordOutput("autotrap2:state", state_) ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
    
    @Override
    public void end(boolean interrupted) {
        db_.stopPath() ;
    }

    private void driveToTrap2State() {
        if (!db_.isFollowingPath()) {
            return ;
        }
    }

    private void rotateState() {
        if (rotate_.isFinished()) {
            rotate_ = null ;
            delay2_timer_.start() ;
            state_ = State.Delay2 ;
        }
    }

    private void driveToTrap1State() {
        if (!db_.isFollowingPath()) {
            state_ = State.Delay1 ;
            delay1_timer_.start() ;
        }
    }


    private void lookForAprilTag() {
        if (XeroRobot.isReal()) {
            //
            // Find the april tag based on what we see
            //
            LimelightResults results = LimelightHelpers.getLatestResults(getLimelightName()) ;
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
            waypoints_ = computeTarget(target_tag_) ;
            if (waypoints_ != null) {
                db_.driveTo("auto-amp", null, waypoints_[0], 1.0, 1.0, 0.0, 1.0, 1.0) ;
                state_ = State.DriveToTrap1 ;
            }
            else {
                state_ = State.Done ;
            }
        }
        else {
            state_ = State.Done ;
        }
    }

    protected Pose2dWithRotation [] computeTarget(int tag) {
        Pose2dWithRotation [] pts = super.computeTarget(tag, kExtra2Spacing1, kRight2Spacing1, kExtra2Spacing2, kRight2Spacing2) ;

        if (pts != null) {
            Rotation2d robotrot = pts[0].getRobotRotation().rotateBy(Rotation2d.fromDegrees(180.0)) ;
            pts[0] = new Pose2dWithRotation(pts[0].getTranslation(), pts[0].getRotation(), robotrot) ;
        }

        return pts ;
    }
}
