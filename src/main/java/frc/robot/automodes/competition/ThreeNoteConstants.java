package frc.robot.automodes.competition;

import java.util.Optional;

import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ThreeNoteConstants extends AutoModeConstants {
    public static final double kLowManualTilt = 6.0 ;
    public static final double kLowManualTiltPosTol = 5.0 ;
    public static final double kLowManualTiltVelTol = 32.0 ;
    public static final double kLowManualUpDown = 35.0 ;
    public static final double kLowManualUpDownPosTol = 5.0 ;
    public static final double kLowManualUpDownVelTol = 5.0 ;
    public static final double kLowManualShooter = 75.0 ;
    public static final double kLowManualShooterVelTol = 10.0 ;

    private static final Pose2dWithRotation kStartPosition = new Pose2dWithRotation(new Pose2d(0.76, 4.37, Rotation2d.fromDegrees(-60.0)), Rotation2d.fromDegrees(-60.0)) ;

    private static final Pose2dWithRotation kCollect2PoseConst = new Pose2dWithRotation(new Pose2d(7.76, 0.42, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect2PoseImdConst = new Pose2dWithRotation(new Pose2d(4.58, 1.13, Rotation2d.fromDegrees(-30.0)), Rotation2d.fromDegrees(0.0)) ;

    private static final Pose2dWithRotation kShoot2ImdConst = new Pose2dWithRotation(new Pose2d(2.19, 3.6, Rotation2d.fromDegrees(130.0)), Rotation2d.fromDegrees(-50.0)) ;
    private static final Pose2dWithRotation kShoot2PoseConst = new Pose2dWithRotation(new Pose2d(2.0, 3.00, Rotation2d.fromDegrees(150.0)), Rotation2d.fromDegrees(-40.0)) ;

    private static final Pose2dWithRotation kCollect3PoseConst = new Pose2dWithRotation(new Pose2d(7.74, 2.20, Rotation2d.fromDegrees(30.0)), Rotation2d.fromDegrees(30.0)) ;
    private static final Pose2dWithRotation kCollect3ImdConst = new Pose2dWithRotation(new Pose2d(4.5, 1.5, Rotation2d.fromDegrees(30.0)), Rotation2d.fromDegrees(30.0)) ;

    private static final Pose2dWithRotation kShoot3PoseConst = new Pose2dWithRotation(new Pose2d(2.54, 3.28, Rotation2d.fromDegrees(150.0)), Rotation2d.fromDegrees(-40.0)) ;
    private static final Pose2dWithRotation kShoot3ImdConst = new Pose2dWithRotation(new Pose2d(4.0, 1.0, Rotation2d.fromDegrees(180.0)), Rotation2d.fromDegrees(-30.0)) ;

    public static Pose2dWithRotation getStartPosition(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kStartPosition : mirror(kStartPosition, length) ;
    }
    
    public static Pose2dWithRotation getCollect2Pose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect2PoseConst : mirror(kCollect2PoseConst, length) ;
    }  

    public static Pose2dWithRotation getCollect2PoseImmd(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect2PoseImdConst : mirror(kCollect2PoseImdConst, length) ;
    }      

    public static Pose2dWithRotation getShooot2ImdPose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShoot2ImdConst : mirror(kShoot2ImdConst, length) ;
    }  
    
    public static Pose2dWithRotation getShoot2Pose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShoot2PoseConst : mirror(kShoot2PoseConst, length) ;
    }    

    public static Pose2dWithRotation getCollect3ImdPose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect3ImdConst : mirror(kCollect3ImdConst, length) ;
    }      

    public static Pose2dWithRotation getCollect3Pose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect3PoseConst : mirror(kCollect3PoseConst, length) ;
    }  
    
    public static Pose2dWithRotation getShoot3Pose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShoot3PoseConst : mirror(kShoot3PoseConst, length) ;
    }        

    public static Pose2dWithRotation getShooot3ImdPose(double length) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShoot3ImdConst : mirror(kShoot3ImdConst, length) ;
    }      
}
