package frc.robot.autos;

import org.xero1425.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AutoModeConstants {
    public final class FourNoteDynamic {
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
}
