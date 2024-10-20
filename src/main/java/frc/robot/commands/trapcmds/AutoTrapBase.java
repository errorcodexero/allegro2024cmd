package frc.robot.commands.trapcmds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.LimelightHelpers.LimelightResults;
import org.xero1425.math.Pose2dWithRotation;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.LEDState;
import frc.robot.subsystems.oi.OISubsystem.OILed;

public class AutoTrapBase extends Command {
    private static double kExtraSpacing1 = 1.5 ;       // Off from the april tag toward the robot
    private static double kExtraSpacing2 = 0.22 ;      // Off from the april tag toward the robot
    private static double kRightSpacing1 = 0.0 ;       // Positive moves to the right
    private static double kRightSpacing2 = 0.05 ;      // Positive moves to the right

    private static int lasttag_ ;                       // The last valid stage tag we have seen

    private CommandSwerveDrivetrain db_ ;
    private AprilTagFieldLayout layout_ ;
    private double maxdist_ ;
    private double mindist_ ;
    private String limelight_ ;

    public AutoTrapBase(CommandSwerveDrivetrain db, String limelight, AprilTagFieldLayout layout, double maxdist, double mindist) {
        db_ = db ;
        limelight_ = limelight ;
        layout_ = layout ;
        maxdist_ = maxdist ;
        mindist_ = mindist ;
    }

    public String getLimelightName() {
        return limelight_ ;
    }

    public static void setLEDs(String limelight_name, AprilTagFieldLayout layout, CommandSwerveDrivetrain dt,  OISubsystem oi) {
        double dist = 0.0 ;

        int [] tags = AutoTrap1Command.getApplicableTags() ;
        if (tags != null) {
            int tag = AutoTrap1Command.seeAprilTag(limelight_name, tags) ;
            if (tag != -1) {
                lasttag_ = tag ;
                Pose2d tagpose = layout.getTagPose(tag).get().toPose2d() ;
                dist = tagpose.getTranslation().getDistance(dt.getState().Pose.getTranslation()) ;
                if (dist < AutoTrap1Command.kMaxDistance && dist > AutoTrap1Command.kMinDistance) {
                    //
                    // We are good to go, just hit the autotrap button to execute.
                    //
                    oi.setLEDState(OILed.AutoTrapExecEnabled, LEDState.On) ;
                }
                else {
                    //
                    // We have a note and we see the april tag, but we are too far away
                    //
                    oi.setLEDState(OILed.AutoTrapExecEnabled, LEDState.Fast) ;
                }
            }
            else {
                Pose2d tagpose = layout.getTagPose(lasttag_).get().toPose2d() ;
                dist = tagpose.getTranslation().getDistance(dt.getState().Pose.getTranslation()) ;

                //
                // Slow blink, we have a note in the trap position, but don't see the april tag
                //
                oi.setLEDState(OILed.AutoTrapExecEnabled, LEDState.Slow) ;
            }

            Logger.recordOutput("autotrap:dist", dist) ;            
        }
    }

    public static int[] getApplicableTags() {
        int [] ret = null ;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            ret = new int[3] ;

            if (alliance.get() == Alliance.Red) {
                ret[0] = 11 ;
                ret[1] = 12 ;
                ret[2] = 13 ;
            }
            else {
                ret[0] = 14 ;
                ret[1] = 15 ;
                ret[2] = 16 ;
            }
        }

        return ret ;
    }

    public static int seeAprilTag(String name, int[] tags) {
        int ret = -1;
        LimelightResults results = LimelightHelpers.getLatestResults(name);

        if (tags != null) {
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                for (int j = 0; j < tags.length; j++) {
                    if (results.targets_Fiducials[i].fiducialID == tags[j]) {
                        ret = tags[j];
                        break;
                    }
                }
            }
        }

        return ret;
    }    

    protected Pose2dWithRotation[] computeTarget(int tag) {
        Pose2dWithRotation[] ret = new Pose2dWithRotation[2];
        Pose2d pt;
        Rotation2d ptrt;

        Pose2d tagpose = layout_.getTagPose(tag).get().toPose2d();

        pt = computeProjectedTrapPoint(tagpose, kExtraSpacing1, kRightSpacing1);
        ptrt = pt.getRotation();
        ret[0] = new Pose2dWithRotation(pt.getTranslation(), ptrt, ptrt);

        pt = computeProjectedTrapPoint(tagpose, kExtraSpacing2, kRightSpacing2);
        ptrt = pt.getRotation();
        ret[1] = new Pose2dWithRotation(pt.getTranslation(), ptrt, ptrt);

        double dist = tagpose.getTranslation().getDistance(db_.getState().Pose.getTranslation());
        if (dist > maxdist_ || dist < mindist_) {
            ret = null;
        }

        return ret;
    }

    protected Translation2d projectPointAlongHeading(Translation2d p, Rotation2d angle, double projection) {
        double dx = Math.cos(angle.getRadians()) * projection;
        double dy = Math.sin(angle.getRadians()) * projection;
        return new Translation2d(p.getX() + dx, p.getY() + dy);
    }

    protected Pose2d computeProjectedTrapPoint(Pose2d tag, double xspacing, double yspacing) {
        // Find the point projected along the heading given by the tag
        Translation2d ret1 = projectPointAlongHeading(tag.getTranslation(), tag.getRotation(), xspacing);

        // Find a point at the end of the offset at the end of the projection
        Translation2d ret2 = projectPointAlongHeading(ret1, tag.getRotation().rotateBy(Rotation2d.fromDegrees(90.0)),
                yspacing);

        // Take the original heading with the final point
        Pose2d ret = new Pose2d(ret2, tag.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)));

        return ret;
    }
}
