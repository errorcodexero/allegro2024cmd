package frc.robot.automodes.competition;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.AllegroContainer;

public class ThreeNoteCommand extends XeroAutoCommand {

    private enum State {
        Start,
        ShootFirstNote,
        DriveToSecondNote,
        DriveToShootSecond,
        ShootingSecondNote,
        Error,
        Done
    } ;

    private State state_ ;
    private AllegroContainer container_ ;
    private Pose2dWithRotation collect2pose_ ;
    private Pose2dWithRotation shoot2pose_ ;
    private Pose2dWithRotation startpose_ ;

    private final static String desc = "This auto mode starts in the center subwoofer position, shoots the loaded note, and collects and shoots the three notes that are near" ;
     
    public ThreeNoteCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());
    }

    @Override
    public void initialize() {
        try {
            collect2pose_ = ThreeNoteConstants.getCollect2Pose(getRobot().getFieldLayout().getFieldLength()) ;
            shoot2pose_ = ThreeNoteConstants.getShoot2Pose(getRobot().getFieldLayout().getFieldLength()) ;            
            startpose_ = ThreeNoteConstants.getStartPosition(getRobot().getFieldLayout().getFieldLength()) ;
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
                    container_.getIntakeShooter().collect() ;
                    container_.getDriveTrain().driveTo("Collect2", null, collect2pose_, 2.0, 2.0, 0, 0, 0.2) ;
                    state_ = State.DriveToSecondNote ;
                }
                break ;

            case DriveToSecondNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        container_.getDriveTrain().driveTo("Shoot2", null, shoot2pose_, 2.0, 2.0, 0, 0, 0.2) ;
                        state_ = State.DriveToShootSecond ;
                    }
                }
                break;

            case DriveToShootSecond:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    state_ = State.ShootingSecondNote ;
                }

            case Error:
                break ;
            case Done:
                break ;
        }
    }    
}
