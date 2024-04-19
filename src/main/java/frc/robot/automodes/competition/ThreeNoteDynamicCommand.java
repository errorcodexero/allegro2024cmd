package frc.robot.automodes.competition;

import org.xero1425.HolonomicPathFollower;
import org.xero1425.Pose2dWithRotation;
import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AllegroContainer;
import frc.robot.NoteDestination;
import frc.robot.automodes.competition.FourNoteDynamicCommand.FourNoteConstants;

public class ThreeNoteDynamicCommand extends XeroAutoCommand {

    public final class ThreeNoteConstants {
        public static final double kLowManualTilt = 3.0 ;
        public static final double kLowManualTiltPosTol = 2.0 ;
        public static final double kLowManualTiltVelTol = 2.0 ;
        public static final double kLowManualUpDown = 35.0 ;
        public static final double kLowManualUpDownPosTol = 5.0 ;
        public static final double kLowManualUpDownVelTol = 5.0 ;
        public static final double kLowManualShooter = 65.0 ;
        public static final double kLowManualShooterVelTol = 5.0 ;
        
        public static final Pose2dWithRotation kP1StartPoseConst = new Pose2dWithRotation(new Pose2d(0.73, 4.27, Rotation2d.fromDegrees(-60.0)), Rotation2d.fromDegrees(-60.0)) ;   
        public static final Pose2dWithRotation kP1Imd1PoseConst = new Pose2dWithRotation(new Pose2d(4.0, 1.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
        public static final Pose2dWithRotation kP1EndPoseConst = new Pose2dWithRotation(new Pose2d(7.63, 0.73, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
        public static final Pose2dWithRotation kP2EndPoseConst = new Pose2dWithRotation(new Pose2d(2.79, 2.87, Rotation2d.fromDegrees(135.0)), Rotation2d.fromDegrees(-45.0)) ;
        public static final Pose2dWithRotation kP3EndPoseConst = new Pose2dWithRotation(new Pose2d(7.62, 2.16, Rotation2d.fromDegrees(30.0)), Rotation2d.fromDegrees(30.0)) ; 
        public static final Pose2dWithRotation kP4EndPoseConst = new Pose2dWithRotation(new Pose2d(2.79, 2.87, Rotation2d.fromDegrees(135.0)), Rotation2d.fromDegrees(-45.0)) ;
    }

    private enum State {
        Start,
        Shoot1,
        DriveToCollect2,
        DriveToShoot2,
        Shoot2,
        DriveToCollect3,
        DriveToShoot3,
        Shoot3,
        Done
    } ;    

    private State state_ ; 
    private AllegroContainer container_ ;

    private final static String desc = "This auto mode starts against the subwoofer on the source side.  It shoots the loaded note and then collects the two notes in the center of the field closest to the source wall and shoots them." ;

    public ThreeNoteDynamicCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());
    }

    @Override
    public void initialize() {
        container_.getDriveTrain().seedFieldRelative(new Pose2d(ThreeNoteConstants.kP1StartPoseConst.getTranslation(), 
                                                            ThreeNoteConstants.kP1StartPoseConst.getRobotRotation()));
        container_.getIntakeShooter().setHasNote(true) ;
        container_.getIntakeShooter().manualShoot(
                ThreeNoteConstants.kLowManualUpDown, ThreeNoteConstants.kLowManualUpDownPosTol, ThreeNoteConstants.kLowManualUpDownVelTol, 
                ThreeNoteConstants.kLowManualTilt, ThreeNoteConstants.kLowManualTiltPosTol, ThreeNoteConstants.kLowManualTiltVelTol,
                ThreeNoteConstants.kLowManualShooter, ThreeNoteConstants.kLowManualShooterVelTol,
                true) ;
        state_ = State.Shoot1 ;        
    }

    @Override
    public void execute() {
        super.execute();

        switch(state_) {
            case Start:
                break ;

            case Shoot1:
                if (!container_.getIntakeShooter().hasNote()) {
                    container_.getOI().setAutoNoteDestination(NoteDestination.AutoSpeaker) ;                    
                    container_.getIntakeShooter().collect() ;
                    Pose2d [] imd = new Pose2d[] { ThreeNoteConstants.kP1Imd1PoseConst } ;
                    getFollower().driveTo("Start-C2", imd, FourNoteConstants.kCollect1PoseConst, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                    state_ = State.DriveToCollect2 ;
                }
                break ;

            case DriveToCollect2:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        getFollower().driveTo("C2-S2", null, ThreeNoteConstants.kP2EndPoseConst, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                        state_ = State.DriveToShoot2 ;                        
                    } else {
                        container_.getIntakeShooter().collect() ;
                        getFollower().driveTo("C2-C3", null, ThreeNoteConstants.kP3EndPoseConst, 3.0, 2.5, 0.0, 0.0, 0.2) ;
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
                    getFollower().driveTo("S2-C3", null, ThreeNoteConstants.kP3EndPoseConst, 3.0, 2.5, 0.0, 0.0, 0.2) ;
                    state_ = State.DriveToCollect3 ;
                }
                break; 

            case DriveToCollect3:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        getFollower().driveTo("C3-S3", null, ThreeNoteConstants.kP4EndPoseConst, 3.0, 2.5, 0.0, 0.0, 0.2) ;
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
        return state_ == State.Done ;
    }    

    private HolonomicPathFollower getFollower() {
        return container_.getDriveTrain().getHolonimicPathFollower() ;
    }    
}
