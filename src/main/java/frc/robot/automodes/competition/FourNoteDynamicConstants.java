package frc.robot.automodes.competition;

import java.util.Optional;

import org.xero1425.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FourNoteDynamicConstants extends AutoModeConstants {

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

    private static final Pose2dWithRotation kShootPoseConstBlue = new Pose2dWithRotation(new Pose2d(1.50, 5.55, Rotation2d.fromDegrees(180.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect1PoseConstBlue = new Pose2dWithRotation(new Pose2d(2.40, 5.55, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect2PoseConstBlue = new Pose2dWithRotation(new Pose2d(2.50, 6.32, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
    private static final Pose2dWithRotation kCollect3PoseConstBlue = new Pose2dWithRotation(new Pose2d(2.30, 4.55, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;        

    private static final Pose2dWithRotation kShootPoseConstRed = new Pose2dWithRotation(new Pose2d(1.50, 5.55, Rotation2d.fromDegrees(180.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect1PoseConstRed = new Pose2dWithRotation(new Pose2d(2.40, 5.55, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;        
    private static final Pose2dWithRotation kCollect2PoseConstRed = new Pose2dWithRotation(new Pose2d(2.50, 6.32, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
    private static final Pose2dWithRotation kCollect3PoseConstRed = new Pose2dWithRotation(new Pose2d(2.30, 4.55, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;

    public static Pose2dWithRotation getShootPose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShootPoseConstBlue : mirror(kShootPoseConstRed, width) ;
    }

    public static Pose2dWithRotation getCollect1Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect1PoseConstBlue : mirror(kCollect1PoseConstRed, width) ;
    }

    public static Pose2dWithRotation getCollect2Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect2PoseConstBlue : mirror(kCollect2PoseConstRed, width) ;
    }

    public static Pose2dWithRotation getCollect3Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect3PoseConstBlue : mirror(kCollect3PoseConstRed, width) ;
    }
}
