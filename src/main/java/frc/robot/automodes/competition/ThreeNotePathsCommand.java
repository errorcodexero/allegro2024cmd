package frc.robot.automodes.competition;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.HolonomicPathFollower;
import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.paths.XeroPath;
import org.xero1425.paths.XeroPathSegment;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AllegroContainer;
import frc.robot.commands.AutoShootCommand;

public class ThreeNotePathsCommand extends XeroAutoCommand {

    private enum State {
        Start,
        ShootFirstNote,
        DriveToSecondNote,
        DriveToShootSecond,
        ShootingSecondNote,
        DriveToThirdNote,
        DriveToShootThird,
        ShootingThirdNote,
        Error,
        Done
    } ;

    private State state_ ;
    private AllegroContainer container_ ;
    private AutoShootCommand shoot_ ;

    private static final double kMaxVelocity = 4.0 ;
    private static final double kMaxAccel = 4.2 ;
    private static final String kAutoModeNamePathsFile = "Side3Auto" ;
    private static final String kCollect2PathName = "Collect2" ;
    private static final String kShoot2PathName = "Shoot2" ;
    private static final String kCollect3PathName = "Collect3" ;
    private static final String kShoot3PathName = "Shoot3" ;

    private XeroPath collect2_blue_path_ ;
    private XeroPath shoot2_blue_path_ ;
    private XeroPath collect3_blue_path_ ;
    private XeroPath shoot3_blue_path_ ;

    private XeroPath collect2_red_path_ ;
    private XeroPath shoot2_red_path_ ;
    private XeroPath collect3_red_path_ ;
    private XeroPath shoot3_red_path_ ;

    private HolonomicPathFollower collect2_ ;
    private HolonomicPathFollower shoot2_ ;
    private HolonomicPathFollower collect3_ ;
    private HolonomicPathFollower shoot3_ ;

    private Optional<Alliance> alliance_ ;

    private final static String desc = "This auto mode starts on the side of the subwoofer and scores 2, collects the third" ;

    public ThreeNotePathsCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note-paths", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain());

        try {
            collect2_blue_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kCollect2PathName + "Blue") ;            
            shoot2_blue_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kShoot2PathName) ;
            collect3_blue_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kCollect3PathName) ;
            shoot3_blue_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kShoot3PathName) ;

            collect2_red_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kCollect2PathName + "Red") ;
            shoot2_red_path_ = new XeroPath(shoot2_blue_path_) ;
            collect3_red_path_ = new XeroPath(collect3_blue_path_) ;
            shoot3_red_path_ = new XeroPath(shoot3_red_path_) ;

            double length = robot.getFieldLayout().getFieldLength() ;            
            collect2_red_path_.mirrorX(length) ;
            shoot2_red_path_.mirrorX(length) ;
            collect3_red_path_.mirrorX(length) ;
            shoot3_red_path_.mirrorX(length) ;

        }
        catch(Exception ex) {
            state_ = State.Error ;
        }
    }

    @Override
    public void setAlliance(Alliance a) {
        CommandSwerveDrivetrain db = container_.getDriveTrain() ;        

        if (a == Alliance.Red) {
            collect2_ = db.createFollower(collect2_red_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
            shoot2_ = db.createFollower(shoot2_red_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
            collect3_ = db.createFollower(collect3_red_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
            shoot3_ = db.createFollower(shoot3_red_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
        }
        else {
            collect2_ = db.createFollower(collect2_blue_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
            shoot2_ = db.createFollower(shoot2_blue_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
            collect3_ = db.createFollower(collect3_blue_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
            shoot3_ = db.createFollower(shoot3_blue_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
        }
    }

    @Override
    public void initialize() {
        if (state_ == State.Error)
            return ;

        //
        // This is ugly and I would definitely design the intake differently knowing what I know now
        //
        container_.getIntakeShooter().setAutoModeAutoShoot(true);

        //
        // Set the position of the robot
        //
        Pose2dWithRotation start = collect2_.getStartPose() ;
        Pose2d p = new Pose2d(start.getX(), start.getY(), start.getRobotRotation()) ;
        container_.getDriveTrain().seedFieldRelative(p) ;

        container_.getIntakeShooter().setHasNote(true) ;
        container_.getIntakeShooter().manualShoot(
                ThreeNoteConstants.kLowManualUpDown, ThreeNoteConstants.kLowManualUpDownPosTol, ThreeNoteConstants.kLowManualUpDownVelTol,
                ThreeNoteConstants.kLowManualTilt, ThreeNoteConstants.kLowManualTiltPosTol, ThreeNoteConstants.kLowManualTiltVelTol,
                ThreeNoteConstants.kLowManualShooter, ThreeNoteConstants.kLowManualShooterVelTol,
                true, true) ;
        state_ = State.ShootFirstNote ;
    }

    @Override
    public void execute() {
        super.execute() ;

        switch(state_) {
            case Start:
                break;

            case ShootFirstNote:
                if (!container_.getIntakeShooter().hasNote()) {
                    container_.getDriveTrain().setPathFollower(collect2_) ;
                    state_ = State.DriveToSecondNote ;
                }
                break ;

            case DriveToSecondNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (!container_.getIntakeShooter().hasNote()) {
                        container_.getIntakeShooter().turtle(true) ;
                    }
                    container_.getDriveTrain().setPathFollower(shoot2_) ;
                    state_ = State.DriveToShootSecond ;
                }
                break;

            case DriveToShootSecond:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter(), false) ;
                        CommandScheduler.getInstance().schedule(shoot_) ;
                        state_ = State.ShootingSecondNote ;
                    }
                    else {
                        container_.getDriveTrain().setPathFollower(collect3_);
                        state_ = State.DriveToThirdNote ;
                    }
                }
                break ;

            case ShootingSecondNote:
                if (shoot_.isFinished()) {
                    container_.getDriveTrain().setPathFollower(collect3_);
                    state_ = State.DriveToThirdNote ;
                }
                break ;

            case DriveToThirdNote:
                container_.getIntakeShooter().collect() ;
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        container_.getDriveTrain().setPathFollower(shoot3_) ;
                        state_ = State.DriveToShootThird ;
                    }
                    else {
                        container_.getIntakeShooter().turtle(true) ;
                        state_ = State.Done ;
                    }
                }
                break ;

            case DriveToShootThird:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter(), false) ;
                    CommandScheduler.getInstance().schedule(shoot_) ;
                    state_ = State.ShootingThirdNote ;
                }
                break ;

            case ShootingThirdNote:
                if (shoot_.isFinished()) {
                    shoot_ = null ;
                    state_ = State.Done ;
                }
                break ;

            case Error:
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("states:auto-three-path", state_) ;
    }

    @Override
    public boolean isFinished() {
        boolean ret = (state_ == State.Done || state_ == State.Error) ;
        return ret;
    }

    @Override
    public void end(boolean interrupted) {
        container_.getDriveTrain().stopPath();
    }
}
