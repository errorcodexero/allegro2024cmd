package frc.robot.commands.trapcmds;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.SwerveRotateToAngle;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

// This version of auto trap looks for an april tag and then rotates the robot 180 
// degrees to face the tag.  It then drives to a point kExtraSpacing2 away from the tag.

public class AutoTrap3Command extends AutoTrapBase {

    private enum State {
        Starting,
        LookForAprilTag,
        RotateDB,
        DriveToTrap,
        Wait,
        Trapping,
        Done
    }

    public static double kMaxDistance = 3.0;
    public static double kMinDistance = 1.75 ;
    private static int kSimulatedTag = 11;

    private OISubsystem oi_;
    private TrampSubsystem tramp_;
    private CommandSwerveDrivetrain db_;
    private SwerveRotateToAngle rotate_cmd_ ;

    private Command trap_command_;
    private State state_;
    private int target_tag_;
    private static int[] desired_tags_ = null;

    public AutoTrap3Command(String limelight, AprilTagFieldLayout layout, OISubsystem oi, TrampSubsystem tramp, CommandSwerveDrivetrain dt) {
        super(dt, limelight, layout, kMaxDistance, kMinDistance);
        db_ = dt;
        oi_ = oi;
        tramp_ = tramp;
        addRequirements(db_);
        state_ = State.Starting;
    }

    @Override
    public void initialize() {
        target_tag_ = -1;

        if (desired_tags_ == null) {
            desired_tags_ = getApplicableTags();
        }

        if (desired_tags_ == null) {
            state_ = State.Done;
        } else {
            state_ = State.LookForAprilTag;
        }
    }

    @Override
    public void execute() {
        switch (state_) {
            case Starting:
                break;
            case LookForAprilTag:
                if (oi_.isAbortPressed()) {
                    db_.stopPath();
                    state_ = State.Done;
                } else {
                    lookForAprilTag();
                }
                break;

            case RotateDB:
                if (rotate_cmd_.isFinished()) {
                    Pose2dWithRotation[] waypoints = computeTarget(target_tag_);
                    rotate_cmd_ = null ;
                    db_.driveTo("autotrap", null, waypoints[1], 1.0, 1.0, 0.0, 0.0, 1.0) ;
                    state_ = State.DriveToTrap ;
                }
                break ;

            case DriveToTrap:
                if (oi_.isAbortPressed()) {
                    db_.stopPath();
                    state_ = State.Done;
                } else if (!db_.isFollowingPath()) {
                    state_ = State.Wait;
                }
                break;

            case Wait:
                if (oi_.isAbortPressed()) {
                    db_.stopPath();
                    state_ = State.Done;
                } else if (oi_.isAutoTrapPressed()) {
                    trap_command_ = tramp_.trapCommand();
                    CommandScheduler.getInstance().schedule(trap_command_);
                    state_ = State.Trapping;
                }
                break;

            case Trapping:
                if (trap_command_.isFinished()) {
                    state_ = State.Done;
                }
                break;

            case Done:
                break;
        }

        Logger.recordOutput("autotrap3:tag", Integer.toString(target_tag_));
        Logger.recordOutput("autotrap3:state", state_);
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done;
    }

    @Override
    public void end(boolean interrupted) {
        db_.stopPath();
    }

    public static int seeAprilTag(String name, int[] tags) {
        int ret = -1;
        LimelightResults results = LimelightHelpers.getLatestResults(name);

        if (tags != null) {
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                for (int j = 0; j < tags.length; j++) {
                    if (results.targets_Fiducials[i].fiducialID == tags[j]) {
                        ret = tags[j];
                        break;
                    }
                }
            }
        }

        return ret;
    }

    private void lookForAprilTag() {
        if (XeroRobot.isReal()) {
            //
            // Find the april tag based on what we see
            //
            target_tag_ = seeAprilTag(getLimelightName(), desired_tags_);
        } else {
            //
            // Force the april tag based on the simulation
            //
            target_tag_ = kSimulatedTag;
        }

        if (target_tag_ != -1) {
            Pose2dWithRotation[] waypoints = computeTarget(target_tag_);
            if (waypoints != null) {
                rotate_cmd_ = new SwerveRotateToAngle(db_, waypoints[0].getRotation()) ;
                state_ = State.RotateDB ;
                CommandScheduler.getInstance().schedule(rotate_cmd_);
            } else {
                state_ = State.Done;
            }
        } else {
            state_ = State.Done;
        }
    }
}
