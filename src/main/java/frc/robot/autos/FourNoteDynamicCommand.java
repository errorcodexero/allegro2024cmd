package frc.robot.autos;

import org.littletonrobotics.junction.Logger;
import org.xero1425.HolonomicPathFollower;
import org.xero1425.XeroRobot;
import org.xero1425.XeroTimer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AllegroContainer;
import frc.robot.NoteDestination;

public class FourNoteDynamicCommand extends AllegroAutoCommand {

    private enum State {
        Start,
        ShootFirst,
        DelayForIntake,
        ShootSecond,        
        ShootThird,        
        ShootFourth,   
        MovingToSecondNote,
        MovingToThirdNote,
        MovingToFourthNote,
        FinishSecond,
        FinishThird,
        FinishFourth,
        Done
    } ;

    private State state_ ;
    private XeroTimer collect_delay_timer_ ;
    
    public FourNoteDynamicCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, container) ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());

        collect_delay_timer_ = new XeroTimer(getRobot(), "four-note-collect-delay", AutoModeConstants.FourNoteDynamic.kDelayForIntakeDownTime) ;
    }

    @Override
    public void initialize() {
        getContainer().getDriveTrain().seedFieldRelative(new Pose2d(AutoModeConstants.FourNoteDynamic.kShootPoseConst.getTranslation(), 
                                                            AutoModeConstants.FourNoteDynamic.kShootPoseConst.getRobotRotation()));
        getContainer().getIntakeShooter().setHasNote(true) ;
        getContainer().getIntakeShooter().manualShoot(
                AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualUpDownPosTol, AutoModeConstants.FourNoteDynamic.kLowManualUpDownVelTol, 
                AutoModeConstants.FourNoteDynamic.kLowManualTilt, AutoModeConstants.FourNoteDynamic.kLowManualTiltPosTol, AutoModeConstants.FourNoteDynamic.kLowManualTiltVelTol,
                AutoModeConstants.FourNoteDynamic.kLowManualShooter, AutoModeConstants.FourNoteDynamic.kLowManualShooterVelTol,
                true) ;
        state_ = State.ShootFirst ;
    }

    @Override
    public void execute() {
        super.execute() ;

        switch(state_) {
            case Start:
                break ;

            case ShootFirst:
                if (!getContainer().getIntakeShooter().hasNote()) {
                    //
                    // Delay for the intake to go down enough that is sure to be
                    // down when the robot arrives at the first note
                    //
                    getContainer().getOI().setAutoNoteDestination(NoteDestination.ManualSpeaker) ;
                    getContainer().getIntakeShooter().setManualShootParameters(AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualTilt) ;
                    getContainer().getIntakeShooter().collect() ;
  
                    collect_delay_timer_.start();
                    state_ = State.DelayForIntake ;
                }
                break ;

            case DelayForIntake:
                if (collect_delay_timer_.isExpired()) {
                    //
                    // Drive to the second note to collect it (the first note we collect)
                    //
                    getFollower().driveTo("Shoot-C2", AutoModeConstants.FourNoteDynamic.kCollect1PoseConst, 3.0, 2.5, 
                                          new Rotation2d(), new Rotation2d(), 0.2) ;
                    state_ = State.MovingToSecondNote ;
                }
                break ;

            case MovingToSecondNote:
                if (!getFollower().isDriving()) {
                    if (getContainer().getIntakeShooter().hasNote()) {
                        //
                        // We have the second note in the robot, drive to the subwoofer to shoot it.
                        //           
                        getFollower().driveTo("C2-Shoot", AutoModeConstants.FourNoteDynamic.kShootPoseConst, 3.0, 2.5, 
                                              new Rotation2d(), new Rotation2d(), 0.2) ;
                        state_ = State.ShootSecond ;
                    }
                    else {
                        //
                        // We missed the second note, skip stright to the third node
                        //
                        getFollower().driveTo("C2-C3", AutoModeConstants.FourNoteDynamic.kCollect2PoseConst, 3.0, 2.5, 
                                              new Rotation2d(), new Rotation2d(), 0.2) ;
                        state_ = State.MovingToThirdNote ;
                    }
                }
                break ;

            case ShootSecond:
                if (getFollower().getDistance() > AutoModeConstants.FourNoteDynamic.kDistanceShoot2) {
                        getContainer().getIntakeShooter().manualShoot(
                                AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualUpDownPosTol, AutoModeConstants.FourNoteDynamic.kLowManualUpDownVelTol, 
                                AutoModeConstants.FourNoteDynamic.kLowManualTilt, AutoModeConstants.FourNoteDynamic.kLowManualTiltPosTol, AutoModeConstants.FourNoteDynamic.kLowManualTiltVelTol,
                                AutoModeConstants.FourNoteDynamic.kLowManualShooter, AutoModeConstants.FourNoteDynamic.kLowManualShooterVelTol, true) ;
                    state_ = State.FinishSecond ;
                }
                break ;

            case FinishSecond:
                if (!getFollower().isDriving() && !getContainer().getIntakeShooter().hasNote()) {
                    //
                    // We finished shooting the second note, so now we go collect the third note (the second note we collect)
                    //
                    getContainer().getOI().setAutoNoteDestination(NoteDestination.ManualSpeaker) ;
                    getContainer().getIntakeShooter().setManualShootParameters(AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualTilt) ;
                    getContainer().getIntakeShooter().collect() ;                    
                    getFollower().driveTo("Shoot-C3", AutoModeConstants.FourNoteDynamic.kCollect2PoseConst, 3.0, 2.5, 
                                          new Rotation2d(), new Rotation2d(), 0.2) ;
                    state_ = State.MovingToThirdNote ;
                }
                break ;

            case MovingToThirdNote:
                if (!getFollower().isDriving()) {
                    if (getContainer().getIntakeShooter().hasNote()) {
                        //
                        // We have the third note in the robot, drive to the subwoofer to shoot it.
                        //       
                        getFollower().driveTo("C3-Shoot", AutoModeConstants.FourNoteDynamic.kShootPoseConst, 3.0, 2.5, 
                                              new Rotation2d(), new Rotation2d(), 0.2) ;                            
                        getContainer().getIntakeShooter().manualShoot(
                                AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualUpDownPosTol, AutoModeConstants.FourNoteDynamic.kLowManualUpDownVelTol, 
                                AutoModeConstants.FourNoteDynamic.kLowManualTilt, AutoModeConstants.FourNoteDynamic.kLowManualTiltPosTol, AutoModeConstants.FourNoteDynamic.kLowManualTiltVelTol,
                                AutoModeConstants.FourNoteDynamic.kLowManualShooter, AutoModeConstants.FourNoteDynamic.kLowManualShooterVelTol, true) ;
                        state_ = State.ShootThird ;                                   

                    }
                    else {
                        //
                        // Missed the third note, skip to the fourth note
                        //
                        getFollower().driveTo("C3-C4", AutoModeConstants.FourNoteDynamic.kCollect3PoseConst, 3.0, 2.5, 
                                              new Rotation2d(), new Rotation2d(), 0.2) ;
                        state_ = State.MovingToFourthNote ;
                    }
                }
                break ;

            case ShootThird:
                if (getFollower().getDistance() > AutoModeConstants.FourNoteDynamic.kDistanceShoot2) {
                    getContainer().getIntakeShooter().manualShoot(
                            AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualUpDownPosTol, AutoModeConstants.FourNoteDynamic.kLowManualUpDownVelTol, 
                            AutoModeConstants.FourNoteDynamic.kLowManualTilt, AutoModeConstants.FourNoteDynamic.kLowManualTiltPosTol, AutoModeConstants.FourNoteDynamic.kLowManualTiltVelTol,
                            AutoModeConstants.FourNoteDynamic.kLowManualShooter, AutoModeConstants.FourNoteDynamic.kLowManualShooterVelTol, true) ;
                    state_ = State.FinishThird ;
                }
                break ;

            case FinishThird:
                if (!getFollower().isDriving() && !getContainer().getIntakeShooter().hasNote()) {
                    //
                    // We finished shooting the second note, so now we go collect the third note (the second note we collect)
                    //
                    getContainer().getOI().setAutoNoteDestination(NoteDestination.ManualSpeaker) ;
                    getContainer().getIntakeShooter().setManualShootParameters(AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualTilt) ;
                    getContainer().getIntakeShooter().collect() ;                    
                    getFollower().driveTo("Shoot-C4", AutoModeConstants.FourNoteDynamic.kCollect3PoseConst, 3.0, 2.5, 
                                          new Rotation2d(), new Rotation2d(), 0.2) ;
                    state_ = State.MovingToFourthNote ;
                }
                break ;
                
            case MovingToFourthNote:
                if (!getFollower().isDriving()) {
                    if (getContainer().getIntakeShooter().hasNote()) {
                        //
                        // We have the third note in the robot, drive to the subwoofer to shoot it.
                        //       
                        getFollower().driveTo("C4-Shoot", AutoModeConstants.FourNoteDynamic.kShootPoseConst, 3.0, 2.5, 
                                              new Rotation2d(), new Rotation2d(), 0.2) ;                            
                        getContainer().getIntakeShooter().manualShoot(
                                AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualUpDownPosTol, AutoModeConstants.FourNoteDynamic.kLowManualUpDownVelTol, 
                                AutoModeConstants.FourNoteDynamic.kLowManualTilt, AutoModeConstants.FourNoteDynamic.kLowManualTiltPosTol, AutoModeConstants.FourNoteDynamic.kLowManualTiltVelTol,
                                AutoModeConstants.FourNoteDynamic.kLowManualShooter, AutoModeConstants.FourNoteDynamic.kLowManualShooterVelTol, true) ;
                        state_ = State.ShootFourth ;                                   

                    }
                    else {
                        state_ = State.Done;
                    }
                }
                break ;

            case ShootFourth:
                if (getFollower().getDistance() > AutoModeConstants.FourNoteDynamic.kDistanceShoot2) {
                        getContainer().getIntakeShooter().manualShoot(
                                AutoModeConstants.FourNoteDynamic.kLowManualUpDown, AutoModeConstants.FourNoteDynamic.kLowManualUpDownPosTol, AutoModeConstants.FourNoteDynamic.kLowManualUpDownVelTol, 
                                AutoModeConstants.FourNoteDynamic.kLowManualTilt, AutoModeConstants.FourNoteDynamic.kLowManualTiltPosTol, AutoModeConstants.FourNoteDynamic.kLowManualTiltVelTol,
                                AutoModeConstants.FourNoteDynamic.kLowManualShooter, AutoModeConstants.FourNoteDynamic.kLowManualShooterVelTol, true) ;

                    state_ = State.FinishFourth ;
                }
                break ;

            case FinishFourth:
                if (!getFollower().isDriving() && !getContainer().getIntakeShooter().hasNote()) {
                    state_ = State.Done;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("four-note-state", state_) ;
    }

    private HolonomicPathFollower getFollower() {
        return getContainer().getDriveTrain().getHolonimicPathFollower() ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
