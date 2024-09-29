package frc.robot.automodes.competition;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.math.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AllegroContainer;

public class DriveStraight extends XeroAutoCommand {

    private enum State {
        Start,
        Driving1,
        Delaying,
        Driving2,
        Done,
    } ;

    private State state_ ;
    private AllegroContainer container_ ;
    private double timer_ ;
    private double velocity_ = 1.0 ;
    private double accel_ = 1.0 ;

    private static final Pose2dWithRotation kStartPosition = new Pose2dWithRotation(new Pose2d(0.621, 4.44, Rotation2d.fromDegrees(-121.4)), Rotation2d.fromDegrees(-121.4)) ;
    private static final Pose2dWithRotation kCollect2PoseConst = new Pose2dWithRotation(new Pose2d(8.2296, 7.3, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(30.0)) ;


    // 394 cm

    private final static String desc = "This auto mode tests a drive straight" ;
    
    public DriveStraight(XeroRobot robot, AllegroContainer container) {
        super(robot, "drive-straight", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());
    }

    public static Pose2dWithRotation getStartPosition(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kStartPosition : mirror(kStartPosition, width) ;
    }    

    @Override
    public void initialize() {
        try {
            container_.getDriveTrain().seedFieldRelative(getStartPosition(16.541)) ;
        } catch (Exception e) {
        }

        Pose2dWithRotation target = mirror(kCollect2PoseConst, 16.541) ;

        container_.getDriveTrain().driveTo("Straight", null, target, 
                    velocity_, accel_, 0, 0, 5.0) ;

        state_ = State.Driving1 ;
    }

    @Override
    public void execute() {
        super.execute() ;

        switch(state_) {
            case Start:
                break ;

            case Driving1:
                if (!container_.getDriveTrain().isFollowingPath())
                {
                    state_ = State.Done ;
                    timer_ = Timer.getFPGATimestamp() ;
                }
                break ;

            case Delaying:
                if (Timer.getFPGATimestamp() - timer_ > 2.0)
                {
                    Pose2dWithRotation target = new Pose2dWithRotation(0.0, 0.0, new Rotation2d(), new Rotation2d()) ;
                    container_.getDriveTrain().driveTo("Straight", null, target, 
                            velocity_, accel_, 0, 0, 5.0) ;                    
                            state_ = State.Driving2 ;
                }
                break ;

            case Driving2:
                if (!container_.getDriveTrain().isFollowingPath())
                {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("drive-straight", state_) ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }

    static protected Pose2dWithRotation mirror(Pose2dWithRotation pose, double fieldwidth) {
        Pose2dWithRotation ret = null ;

        double h = XeroMath.normalizeAngleDegrees(180.0 - pose.getRotation().getDegrees());
        double r = XeroMath.normalizeAngleDegrees(180.0 - pose.getRobotRotation().getDegrees());
        Translation2d t = new Translation2d(fieldwidth - pose.getTranslation().getX(), pose.getTranslation().getY());
        ret = new Pose2dWithRotation(new Pose2d(t, Rotation2d.fromDegrees(h)), Rotation2d.fromDegrees(r));
        return ret;
    }      
}
