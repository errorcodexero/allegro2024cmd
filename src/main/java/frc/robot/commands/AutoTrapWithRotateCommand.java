package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.math.XeroMath;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.SwerveRotateToAngle;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class AutoTrapWithRotateCommand extends Command {

    //
    // April Tags
    //
    // Red: 11, 12, 13
    // Blue: 14, 15, 16
    //

    private enum State {
        Starting,
        LookForAprilTag,
        DriveToTrap1,
        Rotate,
        DriveToTrap2,
        Wait,
        Trapping,
        Done
    }

    private static double kExtraSpacing1 = 1.5 ;
    private static double kExtraSpacing2 = 0.0 ;
    private static double kMaxDistance = 4.0 ;
    private static int kSimulatedTag = 11 ;
    private static double kForwardVelocity = 0.5 ;
    private static double kForwardDistance = 1.5 ;

    private String limelight_name_ ;

    private OISubsystem oi_ ;
    private TrampSubsystem tramp_ ;
    private CommandSwerveDrivetrain db_ ;

    private Command trap_command_ ;
    private AprilTagFieldLayout layout_ ;
    private State state_ ;
    private int target_tag_ ;
    private int[] desired_tags_ = new int[3] ;
    private Pose2dWithRotation [] waypoints_ = null ;
    private SwerveRotateToAngle rotate_ = null ;
    private SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    private Pose2d starting_pose_ ;

    public AutoTrapWithRotateCommand(String name, AprilTagFieldLayout layout, OISubsystem oi, TrampSubsystem tramp, CommandSwerveDrivetrain dt) {
        limelight_name_ = name ;
        db_ = dt ;
        oi_ = oi ;
        tramp_ = tramp ;
        layout_ = layout ;
        addRequirements(db_) ;
        state_ = State.Starting ;
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

            case DriveToTrap2:
                if (oi_.isAbortPressed()) {
                    rotate_.cancel();
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

    private void driveToTrap2State() {
        double dist = db_.getState().Pose.getTranslation().getDistance(starting_pose_.getTranslation());
        if (dist > kForwardDistance) {
            db_.setControl(new SwerveRequest.Idle()) ;
            state_ = State.Wait;
        }
    }

    private void rotateState() {
        if (rotate_.isFinished()) {
            rotate_ = null ;
            state_ = State.Done ;
            starting_pose_ = db_.getState().Pose ;
            db_.setControl(forward.withVelocityX(kForwardVelocity)) ;
        }
    }

    private void driveToTrap1State() {
        if (!db_.isFollowingPath()) {
            Rotation2d angle = waypoints_[0].getRobotRotation().rotateBy(Rotation2d.fromDegrees(180.0)) ;
            rotate_ = new SwerveRotateToAngle(db_, angle, 1.0, 1.0) ;
            state_ = State.Done ;
            CommandScheduler.getInstance().schedule(rotate_);
        }
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
            waypoints_ = computeTarget() ;
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

    private Pose2dWithRotation[] computeTarget() {
        Pose2dWithRotation [] ret = new Pose2dWithRotation[2] ;
        Pose2d pt ;
        Rotation2d ptrt ;

        Pose2d tagpose = layout_.getTagPose(target_tag_).get().toPose2d() ;

        pt = computeProjectedTrapPoint(tagpose, kExtraSpacing1);
        ptrt = pt.getRotation().rotateBy(Rotation2d.fromDegrees(180.0));
        ret[0] = new Pose2dWithRotation(pt.getTranslation(), ptrt, ptrt) ;

        pt = computeProjectedTrapPoint(tagpose, kExtraSpacing2) ;
        ptrt = pt.getRotation() ;
        ret[1] = new Pose2dWithRotation(pt.getTranslation(), ptrt, ptrt) ;

        double dist = ret[0].getTranslation().getDistance(db_.getState().Pose.getTranslation()) ;
        if (dist > kMaxDistance) {
            ret = null ;
        }

        return ret;
    }

    private Pose2d computeProjectedTrapPoint(Pose2d p, double projection) {
        double dx = Math.cos(p.getRotation().getRadians()) * projection ;
        double dy = Math.sin(p.getRotation().getRadians()) * projection ;
        double angle = XeroMath.normalizeAngleDegrees(p.getRotation().getDegrees() + 180.0) ;
        return new Pose2d(p.getX() + dx, p.getY() + dy, Rotation2d.fromDegrees(angle)) ;
    }    
}
