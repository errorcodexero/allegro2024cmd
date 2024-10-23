package frc.robot.automodes.competition;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroTimer;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AllegroContainer;
import frc.robot.commands.AutoShootCommand;

public class FourNoteQuickCommand extends XeroAutoCommand {

    private enum State {
        Start,
        ShootFirstNote,
        DelayForIntake,
        DriveToSecondNote,
        ShootingSecondNote,
        DriveToThirdNote,
        ShootingThirdNote,
        DriveToFourthNote,
        ShootingFourthNote,
        DriveToEnd,
        Done,
        Error
    } ;

    private State state_ ;
    private XeroTimer collect_delay_timer_ ;
    private AllegroContainer container_ ;
    private Pose2dWithRotation shootpose_ ;
    private Pose2dWithRotation collect1pose_ ;
    private Pose2dWithRotation collect2pose_ ;
    private Pose2dWithRotation collect3pose_ ;
    private Pose2dWithRotation endpose_ ;
    private AutoShootCommand shoot_ ;
    private boolean trailing_ ;

    private final static String desc = "This auto mode starts in the center subwoofer position, shoots the loaded note, and collects and shoots the three notes that are near" ;
    
    public FourNoteQuickCommand(XeroRobot robot, AllegroContainer container, boolean trailing) {
        super(robot, "four-note-quick" + (trailing ? "-end" : ""), desc) ;

        container_ = container ;
        state_ = State.Start ;
        trailing_ = trailing ;

        addRequirements(container.getDriveTrain());

        collect_delay_timer_ = new XeroTimer("four-note-collect-delay", FourNoteDynamicConstants.kDelayForIntakeDownTime) ;
    }

    @Override
    public void setAlliance(Alliance a) {
    }    

    @Override
    public void initialize() {
        container_.getIntakeShooter().setAutoModeAutoShoot(true) ;
        
        try {
            shootpose_ = FourNoteDynamicConstants.getShootPose(getRobot().getFieldLayout().getFieldLength()) ;
            collect1pose_ = FourNoteDynamicConstants.getCollect1Pose(getRobot().getFieldLayout().getFieldLength()) ;
            collect2pose_ = FourNoteDynamicConstants.getCollect3QPose(getRobot().getFieldLayout().getFieldLength()) ;
            collect3pose_ = FourNoteDynamicConstants.getCollect2Pose(getRobot().getFieldLayout().getFieldLength()) ;
            endpose_ = FourNoteDynamicConstants.getEndPose(getRobot().getFieldLayout().getFieldLength()) ;
        }
        catch(Exception ex) {
            state_ = State.Error ;
            return ;
        }

        container_.getDriveTrain().seedFieldRelative(new Pose2d(shootpose_.getTranslation(), shootpose_.getRobotRotation())) ;
        container_.getIntakeShooter().setHasNote(true) ;
        container_.getIntakeShooter().manualShoot(
                FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualUpDownPosTol, FourNoteDynamicConstants.kLowManualUpDownVelTol, 
                FourNoteDynamicConstants.kLowManualTilt, FourNoteDynamicConstants.kLowManualTiltPosTol, FourNoteDynamicConstants.kLowManualTiltVelTol,
                FourNoteDynamicConstants.kLowManualShooter, FourNoteDynamicConstants.kLowManualShooterVelTol,
                true, true) ;
        state_ = State.ShootFirstNote ;
    }

    @Override
    public void execute() {
        super.execute() ;

        switch(state_) {
            case Start:
                break ;

            case Error:
                break ;

            case ShootFirstNote:
                if (!container_.getIntakeShooter().hasNote()) {
                    //
                    // Delay for the intake to go down enough that is sure to be
                    // down when the robot arrives at the first note
                    //
                    container_.getIntakeShooter().collect() ;
  
                    collect_delay_timer_.start();
                    state_ = State.DelayForIntake ;
                }
                break ;

            case DelayForIntake:
                if (collect_delay_timer_.isExpired()) {
                    //
                    // Drive to the second note to collect it (the first note we collect)
                    //
                    container_.getDriveTrain().driveTo("Shoot-C2", null, collect1pose_, 3.0, 2.5, 0, 0, 0.2) ;
                    state_ = State.DriveToSecondNote ;
                }
                break ;

            case DriveToSecondNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter()) ;
                        CommandScheduler.getInstance().schedule(shoot_) ;
                        state_ = State.ShootingSecondNote ;                        
                    }
                    else 
                    {
                        //
                        // We missed the second note, skip stright to the third node
                        //
                        container_.getDriveTrain().driveTo("C2-C3", null, collect2pose_, 3.0, 2.5, 0, 0.2, 0.2) ;
                        state_ = State.DriveToThirdNote ;
                    }
                }
                break ;

            case ShootingSecondNote:
                if (shoot_.isFinished()) {
                    container_.getIntakeShooter().collect() ;                    
                    container_.getDriveTrain().driveTo("C2-C3", null, collect2pose_, 3.0, 2.5, 0, 0.2, 0.2) ;
                    state_ = State.DriveToThirdNote ;                    
                }
                break ;

            case DriveToThirdNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter()) ;
                        CommandScheduler.getInstance().schedule(shoot_) ;
                        state_ = State.ShootingThirdNote ;
                    }
                    else {
                        //
                        // Missed the third note, skip to the fourth note
                        //
                        container_.getDriveTrain().driveTo("C3-C4", null, collect3pose_, 3.0, 2.5, 0, 0.2, 0.2) ;
                        state_ = State.DriveToFourthNote ;
                    }
                }
                break ;

            case ShootingThirdNote:
                if (shoot_.isFinished()) {
                    container_.getIntakeShooter().collect() ;                    
                    container_.getDriveTrain().driveTo("C3-C4", null, collect3pose_, 3.0, 2.5, 0, 0.2, 0.2) ;
                    state_ = State.DriveToFourthNote ;                 
                }
                break ;                

            case DriveToFourthNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        shoot_ = new AutoShootCommand(container_.getOI(), container_.getTracker(), container_.getDriveTrain(), container_.getIntakeShooter()) ;
                        CommandScheduler.getInstance().schedule(shoot_) ;
                        state_ = State.ShootingFourthNote ;
                    }
                    else {
                        state_ = State.Done;
                    }
                }
                break ;

            case ShootingFourthNote:
                if (shoot_.isFinished()) {
                    if (trailing_) {
                        container_.getDriveTrain().driveTo("C4-End", null, endpose_, 3.0, 2.5, 0, 0.2, 0.2) ;
                        state_ = State.DriveToEnd ;
                    }
                    else {
                        state_ = State.Done ;
                    }
                }
                break ;

            case DriveToEnd:
                container_.getIntakeShooter().collect() ;             
                if (!container_.getDriveTrain().isFollowingPath()) {
                    state_ = State.Done ;
                }   
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("states: four-quick", state_) ;
    }

    @Override
    public void end(boolean interrupted) {
        container_.getDriveTrain().stopPath();
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || state_ == State.Error ;
    }
}
