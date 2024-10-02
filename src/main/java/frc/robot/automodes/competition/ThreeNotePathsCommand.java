package frc.robot.automodes.competition;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.paths.XeroPath;
import org.xero1425.paths.XeroPathSegment;

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
    private static final double kMaxAccel = 2.0 ;
    private static final String kAutoModeNamePathsFile = "Side3Auto" ;
    private static final String kCollect2PathName = "Collect2" ;
    private static final String kShoot2PathName = "Shoot2" ;
    private static final String kCollect3PathName = "Collect3" ;
    private static final String kShoot3PathName = "Shoot3" ;

    private XeroPath collect2_path_ ;
    private XeroPath shoot2_path_ ;
    private XeroPath collect3_path_ ;
    private XeroPath shoot3_path_ ;

    private final static String desc = "This auto mode starts on the side of the subwoofer and scores 2, collects the third" ;
     
    public ThreeNotePathsCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note-paths", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain());

        try {
            collect2_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kCollect2PathName) ;
            shoot2_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kShoot2PathName) ;
            collect3_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kCollect3PathName) ;
            shoot3_path_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kShoot3PathName) ;

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isEmpty())
                throw new Exception("Trying to initialize an automode before alliance is known") ;

            if (alliance.get() == Alliance.Red) {
                double length = robot.getFieldLayout().getFieldLength() ;
                collect2_path_.mirrorX(length) ;
                shoot2_path_.mirrorX(length) ;
                collect3_path_.mirrorX(length) ;
                shoot3_path_.mirrorX(length) ;
            }
        }
        catch(Exception ex) {
            state_ = State.Error ;
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


        XeroPathSegment seg = collect2_path_.getSegment(0, 0) ;
        container_.getDriveTrain().seedFieldRelative(new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getRotation()))) ;

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
                // maxv, maxa, pre_rot_time, post_rot_time, to) ;
                if (!container_.getIntakeShooter().hasNote()) {
                    container_.getDriveTrain().drivePathWithTraj(collect2_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
                    state_ = State.DriveToSecondNote ;
                }
                break ;

            case DriveToSecondNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    container_.getDriveTrain().drivePathWithTraj(shoot2_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
                    state_ = State.DriveToShootSecond ;
                }
                break;

            case DriveToShootSecond:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter()) ;
                    CommandScheduler.getInstance().schedule(shoot_) ;
                    state_ = State.ShootingSecondNote ;
                }
                break ;

            case ShootingSecondNote:
                if (shoot_.isFinished()) {
                    container_.getDriveTrain().drivePathWithTraj(collect3_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
                    state_ = State.DriveToThirdNote ;
                }
                break ;

            case DriveToThirdNote:
                container_.getIntakeShooter().collect() ;
                if (!container_.getDriveTrain().isFollowingPath()) {
                    container_.getDriveTrain().drivePathWithTraj(shoot3_path_, kMaxVelocity, kMaxAccel, 0.0, 1.0, 0.1) ;
                    state_ = State.DriveToShootThird ;                    
                }
                break ;

            case DriveToShootThird:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter()) ;
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
