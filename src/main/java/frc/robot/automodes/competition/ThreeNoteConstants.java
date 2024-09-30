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
    private static final Pose2dWithRotation kCollect2PoseConst = new Pose2dWithRotation(new Pose2d(7.76, 0.69, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kShoot2PoseConst = new Pose2dWithRotation(new Pose2d(2.64, 3.18, Rotation2d.fromDegrees(150.0)), Rotation2d.fromDegrees(-30.0)) ;
    private static final Pose2dWithRotation kCollect3PoseConst = new Pose2dWithRotation(new Pose2d(7.74, 2.20, Rotation2d.fromDegrees(30.0)), Rotation2d.fromDegrees(30.0)) ;
    private static final Pose2dWithRotation kShoot3PoseConst = new Pose2dWithRotation(new Pose2d(2.64, 3.18, Rotation2d.fromDegrees(150.0)), Rotation2d.fromDegrees(-30.0)) ;   

    public static Pose2dWithRotation getStartPosition(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kStartPosition : mirror(kStartPosition, width) ;
    }
    
    public static Pose2dWithRotation getCollect2Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect2PoseConst : mirror(kCollect2PoseConst, width) ;
    }  
    
    public static Pose2dWithRotation getShoot2Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShoot2PoseConst : mirror(kShoot2PoseConst, width) ;
    }    

    public static Pose2dWithRotation getCollect3Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kCollect3PoseConst : mirror(kCollect3PoseConst, width) ;
    }  
    
    public static Pose2dWithRotation getShoot3Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kShoot3PoseConst : mirror(kShoot3PoseConst, width) ;
    }        
}
