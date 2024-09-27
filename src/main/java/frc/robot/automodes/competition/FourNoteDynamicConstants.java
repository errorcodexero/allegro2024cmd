package frc.robot.automodes.competition;

import java.util.Optional;

import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FourNoteDynamicConstants extends AutoModeConstants {

    public static final double kLowManualTilt = 6.0 ;
    public static final double kLowManualTiltPosTol = 5.0 ;
    public static final double kLowManualTiltVelTol = 32.0 ;
    public static final double kLowManualUpDown = 35.0 ;
    public static final double kLowManualUpDownPosTol = 5.0 ;
    public static final double kLowManualUpDownVelTol = 5.0 ;
    public static final double kLowManualShooter = 75.0 ;
    public static final double kLowManualShooterVelTol = 10.0 ;
    public static final double kDelayForIntakeDownTime = 0.1 ;
    public static final double kDistanceShoot2 = 0.2 ;
    public static final double kDistanceShoot3 = 1.05 ;
    public static final double kDistanceShoot4 = 0.7 ;

    public static final double kPath1Velocity = 3.0 ;
    public static final double kPath1Accel = 2.0 ;

    private static final Pose2dWithRotation kShootPoseConst = new Pose2dWithRotation(new Pose2d(1.50, 5.55, Rotation2d.fromDegrees(180.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect1PoseConst = new Pose2dWithRotation(new Pose2d(2.40, 5.55, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect2PoseConst = new Pose2dWithRotation(new Pose2d(2.40, 6.52, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
    private static final Pose2dWithRotation kCollect3PoseConst = new Pose2dWithRotation(new Pose2d(2.30, 4.55, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;        

    public static Pose2dWithRotation getShootPose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShootPoseConst : mirror(kShootPoseConst, width) ;
    }

    public static Pose2dWithRotation getCollect1Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect1PoseConst : mirror(kCollect1PoseConst, width) ;
    }

    public static Pose2dWithRotation getCollect2Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect2PoseConst : mirror(kCollect2PoseConst, width) ;
    }

    public static Pose2dWithRotation getCollect3Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect3PoseConst : mirror(kCollect3PoseConst, width) ;
    }
}
