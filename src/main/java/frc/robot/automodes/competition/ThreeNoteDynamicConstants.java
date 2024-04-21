package frc.robot.automodes.competition;

import java.util.Optional;

import org.xero1425.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ThreeNoteDynamicConstants extends AutoModeConstants {
    public static final double kLowManualTilt = 3.0 ;
    public static final double kLowManualTiltPosTol = 2.0 ;
    public static final double kLowManualTiltVelTol = 2.0 ;
    public static final double kLowManualUpDown = 35.0 ;
    public static final double kLowManualUpDownPosTol = 5.0 ;
    public static final double kLowManualUpDownVelTol = 5.0 ;
    public static final double kLowManualShooter = 65.0 ;
    public static final double kLowManualShooterVelTol = 5.0 ;
    
    private static final Pose2dWithRotation kP1StartPoseConstBlue = new Pose2dWithRotation(new Pose2d(0.73, 4.27, Rotation2d.fromDegrees(-60.0)), Rotation2d.fromDegrees(-60.0)) ;   
    private static final Pose2dWithRotation kP1Imd1PoseConstBlue = new Pose2dWithRotation(new Pose2d(4.0, 1.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kP1EndPoseConstBlue = new Pose2dWithRotation(new Pose2d(7.63, 0.73, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kP2EndPoseConstBlue = new Pose2dWithRotation(new Pose2d(2.79, 2.87, Rotation2d.fromDegrees(135.0)), Rotation2d.fromDegrees(-45.0)) ;
    private static final Pose2dWithRotation kP3EndPoseConstBlue = new Pose2dWithRotation(new Pose2d(7.62, 2.16, Rotation2d.fromDegrees(30.0)), Rotation2d.fromDegrees(30.0)) ; 
    private static final Pose2dWithRotation kP4EndPoseConstBlue = new Pose2dWithRotation(new Pose2d(2.79, 2.87, Rotation2d.fromDegrees(135.0)), Rotation2d.fromDegrees(-45.0)) ;

    private static final Pose2dWithRotation kP1StartPoseConstRed = new Pose2dWithRotation(new Pose2d(0.73, 4.27, Rotation2d.fromDegrees(-60.0)), Rotation2d.fromDegrees(-60.0)) ;   
    private static final Pose2dWithRotation kP1Imd1PoseConstRed = new Pose2dWithRotation(new Pose2d(4.0, 1.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kP1EndPoseConstRed= new Pose2dWithRotation(new Pose2d(7.63, 0.73, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kP2EndPoseConstRed = new Pose2dWithRotation(new Pose2d(2.79, 2.87, Rotation2d.fromDegrees(135.0)), Rotation2d.fromDegrees(-45.0)) ;
    private static final Pose2dWithRotation kP3EndPoseConstRed = new Pose2dWithRotation(new Pose2d(7.62, 2.16, Rotation2d.fromDegrees(30.0)), Rotation2d.fromDegrees(30.0)) ; 
    private static final Pose2dWithRotation kP4EndPoseConstRed = new Pose2dWithRotation(new Pose2d(2.79, 2.87, Rotation2d.fromDegrees(135.0)), Rotation2d.fromDegrees(-45.0)) ;   
    
    public static Pose2dWithRotation getStartPose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kP1StartPoseConstBlue : mirror(kP1StartPoseConstRed, width) ;
    }

    public static Pose2dWithRotation getImd1Pose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kP1Imd1PoseConstBlue : mirror(kP1Imd1PoseConstRed, width) ;
    }

    public static Pose2dWithRotation getEndPose(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kP1EndPoseConstBlue : mirror(kP1EndPoseConstRed, width) ;
    }

    public static Pose2dWithRotation getEndPose2(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kP2EndPoseConstBlue : mirror(kP2EndPoseConstRed, width) ;
    }

    public static Pose2dWithRotation getEndPose3(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kP3EndPoseConstBlue : mirror(kP3EndPoseConstRed, width) ;
    }

    public static Pose2dWithRotation getEndPose4(double width) throws Exception {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            throw new Exception("Trying to initialize an automode before alliance is known") ;

        return (alliance.get() == Alliance.Blue) ? kP4EndPoseConstBlue : mirror(kP4EndPoseConstRed, width) ;
    }
}
