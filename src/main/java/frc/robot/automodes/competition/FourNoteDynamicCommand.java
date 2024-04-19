package frc.robot.automodes.competition;

import org.littletonrobotics.junction.Logger;
import org.xero1425.HolonomicPathFollower;
import org.xero1425.Pose2dWithRotation;
import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;
import org.xero1425.XeroTimer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AllegroContainer;
import frc.robot.NoteDestination;

public class FourNoteDynamicCommand extends XeroAutoCommand {

    public final class FourNoteConstants {
        public static final double kLowManualTilt = 3.0 ;
        public static final double kLowManualTiltPosTol = 2.0 ;
        public static final double kLowManualTiltVelTol = 2.0 ;
        public static final double kLowManualUpDown = 35.0 ;
        public static final double kLowManualUpDownPosTol = 5.0 ;
        public static final double kLowManualUpDownVelTol = 5.0 ;
        public static final double kLowManualShooter = 65.0 ;
        public static final double kLowManualShooterVelTol = 5.0 ;
        public static final double kDelayForIntakeDownTime = 0.4 ;
        public static final double kDistanceShoot2 = 0.5 ;
        public static final double kDistanceShoot3 = 0.5 ;
        public static final double kDistanceShoot4 = 0.5 ;

        public static final double kPath1Velocity = 3.0 ;
        public static final double kPath1Accel = 2.5 ;

        public static final Pose2dWithRotation kShootPoseConst = new Pose2dWithRotation(new Pose2d(1.50, 5.55, Rotation2d.fromDegrees(180.0)), Rotation2d.fromDegrees(0.0)) ;
        public static final Pose2dWithRotation kCollect1PoseConst = new Pose2dWithRotation(new Pose2d(2.40, 5.55, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
        public static final Pose2dWithRotation kCollect2PoseConst = new Pose2dWithRotation(new Pose2d(2.50, 6.32, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
        public static final Pose2dWithRotation kCollect3PoseConst = new Pose2dWithRotation(new Pose2d(2.30, 4.55, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;        
    }    

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
    private AllegroContainer container_ ;

    private final static String desc = "This auto mode starts in the center subwoofer position, shoots the loaded note, and collects and shoots the three notes that are near" ;
    
    public FourNoteDynamicCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "four-note", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());

        collect_delay_timer_ = new XeroTimer(getRobot(), "four-note-collect-delay", FourNoteConstants.kDelayForIntakeDownTime) ;
    }

    @Override
    public void initialize() {
        container_.getDriveTrain().seedFieldRelative(new Pose2d(FourNoteConstants.kShootPoseConst.getTranslation(), 
                                                            FourNoteConstants.kShootPoseConst.getRobotRotation()));
        container_.getIntakeShooter().setHasNote(true) ;
        container_.getIntakeShooter().manualShoot(
                FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualUpDownPosTol, FourNoteConstants.kLowManualUpDownVelTol, 
                FourNoteConstants.kLowManualTilt, FourNoteConstants.kLowManualTiltPosTol, FourNoteConstants.kLowManualTiltVelTol,
                FourNoteConstants.kLowManualShooter, FourNoteConstants.kLowManualShooterVelTol,
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
                if (!container_.getIntakeShooter().hasNote()) {
                    //
                    // Delay for the intake to go down enough that is sure to be
                    // down when the robot arrives at the first note
                    //
                    container_.getOI().setAutoNoteDestination(NoteDestination.ManualSpeaker) ;
                    container_.getIntakeShooter().setManualShootParameters(FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualTilt) ;
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
                    getFollower().driveTo("Shoot-C2", null, FourNoteConstants.kCollect1PoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                    state_ = State.MovingToSecondNote ;
                }
                break ;

            case MovingToSecondNote:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        //
                        // We have the second note in the robot, drive to the subwoofer to shoot it.
                        //           
                        getFollower().driveTo("C2-Shoot", null, FourNoteConstants.kShootPoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                        state_ = State.ShootSecond ;
                    }
                    else {
                        //
                        // We missed the second note, skip stright to the third node
                        //
                        getFollower().driveTo("C2-C3", null, FourNoteConstants.kCollect2PoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                        state_ = State.MovingToThirdNote ;
                    }
                }
                break ;

            case ShootSecond:
                if (getFollower().getDistance() > FourNoteConstants.kDistanceShoot2) {
                        container_.getIntakeShooter().manualShoot(
                                FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualUpDownPosTol, FourNoteConstants.kLowManualUpDownVelTol, 
                                FourNoteConstants.kLowManualTilt, FourNoteConstants.kLowManualTiltPosTol, FourNoteConstants.kLowManualTiltVelTol,
                                FourNoteConstants.kLowManualShooter, FourNoteConstants.kLowManualShooterVelTol, true) ;
                    state_ = State.FinishSecond ;
                }
                break ;

            case FinishSecond:
                if (!getFollower().isDriving() && !container_.getIntakeShooter().hasNote()) {
                    //
                    // We finished shooting the second note, so now we go collect the third note (the second note we collect)
                    //
                    container_.getOI().setAutoNoteDestination(NoteDestination.ManualSpeaker) ;
                    container_.getIntakeShooter().setManualShootParameters(FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualTilt) ;
                    container_.getIntakeShooter().collect() ;                    
                    getFollower().driveTo("Shoot-C3", null, FourNoteConstants.kCollect2PoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                    state_ = State.MovingToThirdNote ;
                }
                break ;

            case MovingToThirdNote:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        //
                        // We have the third note in the robot, drive to the subwoofer to shoot it.
                        //       
                        getFollower().driveTo("C3-Shoot", null, FourNoteConstants.kShootPoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                        container_.getIntakeShooter().manualShoot(
                                FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualUpDownPosTol, FourNoteConstants.kLowManualUpDownVelTol, 
                                FourNoteConstants.kLowManualTilt, FourNoteConstants.kLowManualTiltPosTol, FourNoteConstants.kLowManualTiltVelTol,
                                FourNoteConstants.kLowManualShooter, FourNoteConstants.kLowManualShooterVelTol, true) ;
                        state_ = State.ShootThird ;                                   

                    }
                    else {
                        //
                        // Missed the third note, skip to the fourth note
                        //
                        getFollower().driveTo("C3-C4", null, FourNoteConstants.kCollect3PoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                        state_ = State.MovingToFourthNote ;
                    }
                }
                break ;

            case ShootThird:
                if (getFollower().getDistance() > FourNoteConstants.kDistanceShoot2) {
                    container_.getIntakeShooter().manualShoot(
                            FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualUpDownPosTol, FourNoteConstants.kLowManualUpDownVelTol, 
                            FourNoteConstants.kLowManualTilt, FourNoteConstants.kLowManualTiltPosTol, FourNoteConstants.kLowManualTiltVelTol,
                            FourNoteConstants.kLowManualShooter, FourNoteConstants.kLowManualShooterVelTol, true) ;
                    state_ = State.FinishThird ;
                }
                break ;

            case FinishThird:
                if (!getFollower().isDriving() && !container_.getIntakeShooter().hasNote()) {
                    //
                    // We finished shooting the second note, so now we go collect the third note (the second note we collect)
                    //
                    container_.getOI().setAutoNoteDestination(NoteDestination.ManualSpeaker) ;
                    container_.getIntakeShooter().setManualShootParameters(FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualTilt) ;
                    container_.getIntakeShooter().collect() ;                    
                    getFollower().driveTo("Shoot-C4", null, FourNoteConstants.kCollect3PoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                    state_ = State.MovingToFourthNote ;
                }
                break ;
                
            case MovingToFourthNote:
                if (!getFollower().isDriving()) {
                    if (container_.getIntakeShooter().hasNote()) {
                        //
                        // We have the third note in the robot, drive to the subwoofer to shoot it.
                        //       
                        getFollower().driveTo("C4-Shoot", null, FourNoteConstants.kShootPoseConst, 3.0, 2.5, 0, 0, 0.2) ;
                        container_.getIntakeShooter().manualShoot(
                                FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualUpDownPosTol, FourNoteConstants.kLowManualUpDownVelTol, 
                                FourNoteConstants.kLowManualTilt, FourNoteConstants.kLowManualTiltPosTol, FourNoteConstants.kLowManualTiltVelTol,
                                FourNoteConstants.kLowManualShooter, FourNoteConstants.kLowManualShooterVelTol, true) ;
                        state_ = State.ShootFourth ;                                   

                    }
                    else {
                        state_ = State.Done;
                    }
                }
                break ;

            case ShootFourth:
                if (getFollower().getDistance() > FourNoteConstants.kDistanceShoot2) {
                        container_.getIntakeShooter().manualShoot(
                                FourNoteConstants.kLowManualUpDown, FourNoteConstants.kLowManualUpDownPosTol, FourNoteConstants.kLowManualUpDownVelTol, 
                                FourNoteConstants.kLowManualTilt, FourNoteConstants.kLowManualTiltPosTol, FourNoteConstants.kLowManualTiltVelTol,
                                FourNoteConstants.kLowManualShooter, FourNoteConstants.kLowManualShooterVelTol, true) ;

                    state_ = State.FinishFourth ;
                }
                break ;

            case FinishFourth:
                if (!getFollower().isDriving() && !container_.getIntakeShooter().hasNote()) {
                    state_ = State.Done;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("four-note-state", state_) ;
    }

    private HolonomicPathFollower getFollower() {
        return container_.getDriveTrain().getHolonimicPathFollower() ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
