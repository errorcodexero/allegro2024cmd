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
    private static final double kPathVelocity = 3.0 ;
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
    private Pose2dWithRotation collect2poseimmd_ ;    
    private Pose2dWithRotation shoot2pose_ ;
    private Pose2dWithRotation shoot2imdpose_ ;
    private AutoShootCommand shoot_ ;

    private Pose2dWithRotation startpose_ ;
    private Pose2dWithRotation collect3pose_ ;
    private Pose2dWithRotation collect3imdpose_ ;
    private Pose2dWithRotation shoot3pose_ ;    
    private Pose2dWithRotation shoot3imdpose_;

    private final static String desc = "This auto mode starts in the center subwoofer position, shoots the loaded note, and collects and shoots the three notes that are near" ;
     
    public ThreeNoteCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "three-note", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain());
    }

    @Override
    public void initialize() {
        Logger.recordMetadata("automode", getName()) ;
        //
        // This is ugly and I would definitely design the intake differently knowing what I know now
        //
        container_.getIntakeShooter().setAutoModeAutoShoot(true);
                
        try {
            double length = getRobot().getFieldLayout().getFieldLength() ;
            startpose_ = ThreeNoteConstants.getStartPosition(length) ;            
            collect2pose_ = ThreeNoteConstants.getCollect2Pose(length) ;
            collect2poseimmd_ = ThreeNoteConstants.getCollect2PoseImmd(length) ;
            shoot2pose_ = ThreeNoteConstants.getShoot2Pose(length) ;            
            shoot2imdpose_ = ThreeNoteConstants.getShooot2ImdPose(length);
            collect3pose_ = ThreeNoteConstants.getCollect3Pose(length) ;
            collect3imdpose_ = ThreeNoteConstants.getCollect3ImdPose(length) ;
            shoot3pose_ = ThreeNoteConstants.getShoot3Pose(length) ;
            shoot3imdpose_ = ThreeNoteConstants.getShooot3ImdPose(length);            
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
                    Pose2d immds[] = new Pose2d[] { collect2poseimmd_ } ;                    
                    container_.getDriveTrain().driveTo("Collect2", immds, collect2pose_, 
                                                    kPathVelocity, kPathAcceleration, 0, 1.0, 0.2) ;
                    state_ = State.DriveToSecondNote ;
                }
                break ;

            case DriveToSecondNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        Pose2d immds[] = new Pose2d[] { shoot2imdpose_ } ;
                        container_.getDriveTrain().driveTo("Shoot2", immds, shoot2pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                        state_ = State.DriveToShootSecond ;
                    }
                    else {
                        //
                        // May need imd points here to swing around enough to get the note
                        //
                        container_.getDriveTrain().driveTo("Collect3", null, collect3pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                        state_ = State.DriveToThirdNote ;
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
                    Pose2d immds[] = new Pose2d[] { collect3imdpose_ } ;                    
                    container_.getDriveTrain().driveTo("Collect3", immds, collect3pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
                    state_ = State.DriveToThirdNote ;
                }
                break ;

            case DriveToThirdNote:
                container_.getIntakeShooter().collect() ;
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        Pose2d immds[] = new Pose2d[] { shoot3imdpose_ } ;                          
                        container_.getDriveTrain().driveTo("Shoot3", immds, shoot3pose_, kPathVelocity, kPathAcceleration, 0, 0, 0.2) ;
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
        container_.getDriveTrain().stopPath();
    }
}
