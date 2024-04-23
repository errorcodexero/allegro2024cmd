package frc.robot.automodes.competition;


import org.xero1425.Pose2dWithRotation;
import org.xero1425.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoModeConstants {
    static protected Pose2dWithRotation mirror(Pose2dWithRotation pose, double fieldwidth) {
        Pose2dWithRotation ret = null ;

        double h = XeroMath.normalizeAngleDegrees(180.0 - pose.getRotation().getDegrees());
        double r = XeroMath.normalizeAngleDegrees(180.0 - pose.getRobotRotation().getDegrees());
        Translation2d t = new Translation2d(fieldwidth - pose.getTranslation().getX(), pose.getTranslation().getY());
        ret = new Pose2dWithRotation(new Pose2d(t, Rotation2d.fromDegrees(h)), Rotation2d.fromDegrees(r));
        return ret;
    }    
}
