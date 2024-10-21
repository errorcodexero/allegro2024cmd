package frc.robot.commands.trapcmds;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

//
// This version of auto trap looks for an april tag and then creates a path based on a first
// point with the desired rotation at kExtraSpacing1 and kRightSpacing1 from the tag and a second
// point with the desired rotation at kExtraSpacing2 and kRightSpacing2 from the tag.  This approach
// with two point ensures the robot have finished rotating before it moves under the chain.  The robot
// then waits for the driver to press the auto trap button to start the trapping process.
//

public class AutoTrap1Command extends AutoTrapBase {

    private enum State {
        Starting,
        LookForAprilTag,
        DriveToTrap,
        Wait,
        Trapping,
        Done
    }

    public static double kMaxDistance = 2.5 ;
    public static double kMinDistance = 1.5 ;
    private static int kSimulatedTag = 11 ;

    private OISubsystem oi_ ;
    private TrampSubsystem tramp_ ;
    private CommandSwerveDrivetrain db_ ;

    private Command trap_command_ ;
    private State state_ ;
    private int target_tag_ ;
    private static int[] desired_tags_ = null ;

    public AutoTrap1Command(String limelight, AprilTagFieldLayout layout, OISubsystem oi, TrampSubsystem tramp, CommandSwerveDrivetrain dt) {
        super(dt, limelight, layout, kMaxDistance, kMinDistance) ;

        db_ = dt ;
        oi_ = oi ;
        tramp_ = tramp ;
        addRequirements(db_) ;
        state_ = State.Starting ;

        computeTarget(11) ;
    }

    @Override
    public void initialize() {
        target_tag_ = -1 ;

        if (desired_tags_ == null) {
            desired_tags_ = getApplicableTags() ;
        }

        if (desired_tags_ == null) {
            state_ = State.Done ;
        }
        else {
            state_ = State.LookForAprilTag ;
        }
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

        Logger.recordOutput("autotrap1:tag", Integer.toString(target_tag_)) ;
        Logger.recordOutput("autotrap1:state", state_) ;
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
            target_tag_ = seeAprilTag(getLimelightName(), desired_tags_) ;
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
