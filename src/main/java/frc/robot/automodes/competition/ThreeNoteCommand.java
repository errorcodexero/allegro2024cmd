package frc.robot.automodes.competition;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AllegroContainer;
import frc.robot.commands.AutoShootCommand;

public class ThreeNoteCommand extends XeroAutoCommand {
    private static final double kPathVelocity = 4.0 ;
    private static final double kPathAcceleration = 2.0 ;

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
    private Pose2dWithRotation collect2pose_ ;
    private Pose2dWithRotation shoot2pose_ ;
    private AutoShootCommand shoot_ ;

    private Pose2dWithRotation startpose_ ;
    private Pose2dWithRotation collect3pose_ ;
    private Pose2dWithRotation shoot3pose_ ;    

    private final static String desc = "This auto mode starts in the center subwoofer position, shoots the loaded note, and collects and shoots the three notes that are near" ;
     
    public ThreeNoteCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note", desc) ;

        container_ = container ;
        state_ = State.Start ;
    }

    @Override
    public void initialize() {
        Logger.recordMetadata("automode", getName()) ;
        //
        // This is ugly and I would definitely design the intake differently knowing what I know now
        //
        container_.getIntakeShooter().setAutoModeAutoShoot(true);
                
        try {
            startpose_ = ThreeNoteConstants.getStartPosition(getRobot().getFieldLayout().getFieldLength()) ;            
            collect2pose_ = ThreeNoteConstants.getCollect2Pose(getRobot().getFieldLayout().getFieldLength()) ;
            shoot2pose_ = ThreeNoteConstants.getShoot2Pose(getRobot().getFieldLayout().getFieldLength()) ;            
            collect3pose_ = ThreeNoteConstants.getCollect3Pose(getRobot().getFieldLayout().getFieldLength()) ;
            shoot3pose_ = ThreeNoteConstants.getShoot3Pose(getRobot().getFieldLayout().getFieldLength()) ;
        }
        catch(Exception ex) {
            state_ = State.Error ;
            return ;
        }

        container_.getDriveTrain().seedFieldRelative(new Pose2d(startpose_.getTranslation(), startpose_.getRobotRotation())) ;

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
                    //
                    // Delay for the intake to go down enough that is sure to be
                    // down when the robot arrives at the first note
                    //
                    // container_.getIntakeShooter().collect() ;
                    container_.getDriveTrain().driveTo("Collect2", null, collect2pose_, 
                                                    kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                    state_ = State.DriveToSecondNote ;
                }
                break ;

            case DriveToSecondNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        container_.getDriveTrain().driveTo("Shoot2", null, shoot2pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                        state_ = State.DriveToShootSecond ;
                    }
                    else {
                        //
                        // May need imd points here to swing around enough to get the note
                        //
                        container_.getDriveTrain().driveTo("Shoot3", null, shoot3pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                        state_ = State.DriveToShootThird ;
                    }
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
                    shoot_ = null ;
                    container_.getIntakeShooter().collect() ;
                    container_.getDriveTrain().driveTo("Collect3", null, collect3pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                    state_ = State.DriveToThirdNote ;
                }
                break ;

            case DriveToThirdNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        container_.getDriveTrain().driveTo("Shoot3", null, shoot3pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                        state_ = State.DriveToShootThird ;
                    }
                    else {
                        state_ = State.Done ;
                    }
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

        Logger.recordOutput("states:auto-three", state_) ;
    }    

    @Override
    public boolean isFinished() {
        boolean ret = (state_ == State.Done || state_ == State.Error) ;
        return ret;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("auto-three done") ; 
    }
}
