package frc.robot.automodes.competition;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroTimer;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.AllegroContainer;

public class FourNoteDynamicCommand extends XeroAutoCommand {

    private enum State {
        Start,
        ShootFirstNote,
        DelayForIntake,
        DriveToShootSecond,        
        DriveToShootThirdNote,        
        DriveToShootFourth,   
        DriveToSecondNote,
        DriveToThirdNote,
        DriveToFourthNote,
        FinishSecond,
        FinishThird,
        FinishFourth,
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

    private final static String desc = "This auto mode starts in the center subwoofer position, shoots the loaded note, and collects and shoots the three notes that are near" ;
    
    public FourNoteDynamicCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "four-note", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());

        collect_delay_timer_ = new XeroTimer("four-note-collect-delay", FourNoteDynamicConstants.kDelayForIntakeDownTime) ;
    }

    @Override
    public void initialize() {
        try {
            shootpose_ = FourNoteDynamicConstants.getShootPose(getRobot().getFieldLayout().getFieldWidth()) ;
            collect1pose_ = FourNoteDynamicConstants.getCollect1Pose(getRobot().getFieldLayout().getFieldWidth()) ;
            collect2pose_ = FourNoteDynamicConstants.getCollect2Pose(getRobot().getFieldLayout().getFieldWidth()) ;
            collect3pose_ = FourNoteDynamicConstants.getCollect3Pose(getRobot().getFieldLayout().getFieldWidth()) ;
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
                    container_.getIntakeShooter().setManualShootParameters(FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualTilt) ;
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
                        //
                        // We have the second note in the robot, drive to the subwoofer to shoot it.
                        //           
                        container_.getDriveTrain().driveTo("C2-Shoot", null, shootpose_, 3.0, 2.5, 0, 0, 0.2) ;
                        state_ = State.DriveToShootSecond ;
                    }
                    else 
                    {
                        //
                        // We missed the second note, skip stright to the third node
                        //
                        container_.getDriveTrain().driveTo("C2-C3", null, collect2pose_, 3.0, 2.5, 0, 0, 0.2) ;
                        state_ = State.DriveToThirdNote ;
                    }
                }
                break ;

            case DriveToShootSecond:
                if (container_.getDriveTrain().getPathDistance() > FourNoteDynamicConstants.kDistanceShoot2) {
                        container_.getIntakeShooter().manualShoot(
                                FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualUpDownPosTol, FourNoteDynamicConstants.kLowManualUpDownVelTol, 
                                FourNoteDynamicConstants.kLowManualTilt, FourNoteDynamicConstants.kLowManualTiltPosTol, FourNoteDynamicConstants.kLowManualTiltVelTol,
                                FourNoteDynamicConstants.kLowManualShooter, FourNoteDynamicConstants.kLowManualShooterVelTol, true, true) ;
                    state_ = State.FinishSecond ;
                }
                break ;

            case FinishSecond:
                if (!container_.getDriveTrain().isFollowingPath() && !container_.getIntakeShooter().hasNote()) {
                    //
                    // We finished shooting the second note, so now we go collect the third note (the second note we collect)
                    //
                    container_.getIntakeShooter().setManualShootParameters(FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualTilt) ;
                    container_.getIntakeShooter().collect() ;  
                    container_.getDriveTrain().driveTo("Shoot-C3", null, collect2pose_, 3.0, 2.5, 0, 0, 0.2) ;
                    state_ = State.DriveToThirdNote ;
                }
                break ;

            case DriveToThirdNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        //
                        // We have the third note in the robot, drive to the subwoofer to shoot it.
                        //       
                        container_.getDriveTrain().driveTo("C3-Shoot", null, shootpose_, 3.0, 2.5, 0, 0, 0.2) ;
                        container_.getIntakeShooter().manualShoot(
                                FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualUpDownPosTol, FourNoteDynamicConstants.kLowManualUpDownVelTol, 
                                FourNoteDynamicConstants.kLowManualTilt, FourNoteDynamicConstants.kLowManualTiltPosTol, FourNoteDynamicConstants.kLowManualTiltVelTol,
                                FourNoteDynamicConstants.kLowManualShooter, FourNoteDynamicConstants.kLowManualShooterVelTol, true, true) ;
                        state_ = State.DriveToShootThirdNote ;                                   

                    }
                    else {
                        //
                        // Missed the third note, skip to the fourth note
                        //
                        container_.getDriveTrain().driveTo("C3-C4", null, collect3pose_, 3.0, 2.5, 0, 0, 0.2) ;
                        state_ = State.DriveToFourthNote ;
                    }
                }
                break ;

            case DriveToShootThirdNote:
                if (container_.getDriveTrain().getPathDistance() > FourNoteDynamicConstants.kDistanceShoot2) {
                    container_.getIntakeShooter().manualShoot(
                            FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualUpDownPosTol, FourNoteDynamicConstants.kLowManualUpDownVelTol, 
                            FourNoteDynamicConstants.kLowManualTilt, FourNoteDynamicConstants.kLowManualTiltPosTol, FourNoteDynamicConstants.kLowManualTiltVelTol,
                            FourNoteDynamicConstants.kLowManualShooter, FourNoteDynamicConstants.kLowManualShooterVelTol, true, true) ;
                    state_ = State.FinishThird ;
                }
                break ;

            case FinishThird:
                if (!container_.getDriveTrain().isFollowingPath() && !container_.getIntakeShooter().hasNote()) {
                    //
                    // We finished shooting the second note, so now we go collect the third note (the second note we collect)
                    //
                    container_.getIntakeShooter().setManualShootParameters(FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualTilt) ;
                    container_.getIntakeShooter().collect() ;                    
                    container_.getDriveTrain().driveTo("Shoot-C4", null, collect3pose_, 3.0, 2.5, 0, 0, 0.2) ;
                    state_ = State.DriveToFourthNote ;
                }
                break ;
                
            case DriveToFourthNote:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        //
                        // We have the third note in the robot, drive to the subwoofer to shoot it.
                        //       
                        container_.getDriveTrain().driveTo("C4-Shoot", null, shootpose_, 3.0, 2.5, 0, 0, 0.2) ;
                        container_.getIntakeShooter().manualShoot(
                                FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualUpDownPosTol, FourNoteDynamicConstants.kLowManualUpDownVelTol, 
                                FourNoteDynamicConstants.kLowManualTilt, FourNoteDynamicConstants.kLowManualTiltPosTol, FourNoteDynamicConstants.kLowManualTiltVelTol,
                                FourNoteDynamicConstants.kLowManualShooter, FourNoteDynamicConstants.kLowManualShooterVelTol, true, true);
                        state_ = State.DriveToShootFourth ;                                   

                    }
                    else {
                        state_ = State.Done;
                    }
                }
                break ;

            case DriveToShootFourth:
                if (container_.getDriveTrain().getPathDistance() > FourNoteDynamicConstants.kDistanceShoot2) {
                        container_.getIntakeShooter().manualShoot(
                                FourNoteDynamicConstants.kLowManualUpDown, FourNoteDynamicConstants.kLowManualUpDownPosTol, FourNoteDynamicConstants.kLowManualUpDownVelTol, 
                                FourNoteDynamicConstants.kLowManualTilt, FourNoteDynamicConstants.kLowManualTiltPosTol, FourNoteDynamicConstants.kLowManualTiltVelTol,
                                FourNoteDynamicConstants.kLowManualShooter, FourNoteDynamicConstants.kLowManualShooterVelTol, true, false) ;

                    state_ = State.FinishFourth ;
                }
                break ;

            case FinishFourth:
                if (!container_.getDriveTrain().isFollowingPath() && !container_.getIntakeShooter().hasNote()) {
                    state_ = State.Done;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("four-note-state", state_) ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done || state_ == State.Error ;
    }
}
