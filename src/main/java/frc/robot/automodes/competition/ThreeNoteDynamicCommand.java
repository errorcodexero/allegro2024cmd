package frc.robot.automodes.competition;

import org.xero1425.HolonomicPathFollower;
import org.xero1425.Pose2dWithRotation;
import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.AllegroContainer;
import frc.robot.NoteDestination;

public class ThreeNoteDynamicCommand extends XeroAutoCommand {

    private enum State {
        Start,
        Shoot1,
        DriveToCollect2,
        DriveToShoot2,
        Shoot2,
        DriveToCollect3,
        DriveToShoot3,
        Shoot3,
        Done,
        Error
    } ;    

    private State state_ ; 
    private AllegroContainer container_ ;
    private Pose2dWithRotation start1_ ;
    private Pose2dWithRotation imd1_ ;
    private Pose2dWithRotation end1_ ;
    private Pose2dWithRotation end2_ ;
    private Pose2dWithRotation end3_ ;
    private Pose2dWithRotation end4_ ;

    private final static String desc = "This auto mode starts against the subwoofer on the source side.  It shoots the loaded note and then collects the two notes in the center of the field closest to the source wall and shoots them." ;

    public ThreeNoteDynamicCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());
    }

    @Override
    public void initialize() {
        try {
            start1_ = ThreeNoteDynamicConstants.getStartPose(getRobot().getFieldLayout().getFieldWidth()) ;
            imd1_ = ThreeNoteDynamicConstants.getImd1Pose(getRobot().getFieldLayout().getFieldWidth()) ;
            end1_ = ThreeNoteDynamicConstants.getEndPose(getRobot().getFieldLayout().getFieldWidth()) ;
            end2_ = ThreeNoteDynamicConstants.getEndPose2(getRobot().getFieldLayout().getFieldWidth()) ;
            end3_ = ThreeNoteDynamicConstants.getEndPose3(getRobot().getFieldLayout().getFieldWidth()) ;
            end4_ = ThreeNoteDynamicConstants.getEndPose4(getRobot().getFieldLayout().getFieldWidth()) ;
        }
        catch(Exception ex) {
            state_ = State.Error ;
            return ;
        }

        container_.getDriveTrain().seedFieldRelative(new Pose2d(start1_.getTranslation(), start1_. getRobotRotation())) ; 
        container_.getIntakeShooter().setHasNote(true) ;
        container_.getIntakeShooter().manualShoot(
                ThreeNoteDynamicConstants.kLowManualUpDown, ThreeNoteDynamicConstants.kLowManualUpDownPosTol, ThreeNoteDynamicConstants.kLowManualUpDownVelTol, 
                ThreeNoteDynamicConstants.kLowManualTilt, ThreeNoteDynamicConstants.kLowManualTiltPosTol, ThreeNoteDynamicConstants.kLowManualTiltVelTol,
                ThreeNoteDynamicConstants.kLowManualShooter, ThreeNoteDynamicConstants.kLowManualShooterVelTol,
                true) ;
        state_ = State.Shoot1 ;        
    }

    @Override
    public void execute() {
        super.execute();

        switch(state_) {
            case Start:
                break ;
            case Error:
                break ;

            case Shoot1:
                if (!container_.getIntakeShooter().hasNote()) {
                    container_.getOI().setAutoNoteDestination(NoteDestination.AutoSpeaker) ;                    
                    container_.getIntakeShooter().collect() ;
                    Pose2d [] imd = new Pose2d[] { imd1_ } ;
                    getFollower().driveTo("Start-C2", imd, end1_, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                    state_ = State.DriveToCollect2 ;
                }
                break ;

            case DriveToCollect2:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        getFollower().driveTo("C2-S2", null, end2_, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                        state_ = State.DriveToShoot2 ;                        
                    } else {
                        container_.getIntakeShooter().collect() ;
                        getFollower().driveTo("C2-C3", null, end3_, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                        state_ = State.DriveToCollect3 ;
                    }
                }
                break ;

            case DriveToShoot2:
                if (!getFollower().isDriving()) {
                    container_.getIntakeShooter().finishShot();
                    state_ = State.Shoot2 ;
                }
                break ;

            case Shoot2:
                if (!container_.getIntakeShooter().hasNote()) {
                    getFollower().driveTo("S2-C3", null, end3_, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                    state_ = State.DriveToCollect3 ;
                }
                break; 

            case DriveToCollect3:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        getFollower().driveTo("C3-S3", null, end4_, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                        state_ = State.DriveToShoot3 ;                        
                    } else {
                        state_ = State.Done ;
                    }  
                }          
                break ;

            case DriveToShoot3:
                if (!getFollower().isDriving()) {
                    container_.getIntakeShooter().finishShot();
                    state_ = State.Shoot3 ;
                }
                break ;

            case Shoot3:
                if (!container_.getIntakeShooter().hasNote()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || state_ == State.Error ;
    }    

    private HolonomicPathFollower getFollower() {
        return container_.getDriveTrain().getHolonimicPathFollower() ;
    }    
}
