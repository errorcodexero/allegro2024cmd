package frc.robot.automodes.competition;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AllegroContainer;

public class DriveStraight extends XeroAutoCommand {
    
    private final static String desc = "This auto mode drives straight after a 10 second delay" ;
    private final static double maxv = 1.0 ;
    private final static double maxa = 1.0 ;

    private AllegroContainer container_ ;
    private boolean done_ ;
    
    public DriveStraight(XeroRobot robot, AllegroContainer container) {
        super(robot, "drive-straight", desc) ;

        container_ = container ;
        addRequirements(container.getDriveTrain());
    }

    @Override
    public void initialize() {
        done_ = false ;
        container_.getDriveTrain().seedFieldRelative(new Pose2d()) ;

        // Drive straight
        //Pose2dWithRotation dest = new Pose2dWithRotation(3.89, 0.0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) ;

        // Drive straight with rotation
        // Pose2dWithRotation dest = new Pose2dWithRotation(3.89, 0.0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) ;

        // Drive curve

        //Pose2dWithRotation dest = new Pose2dWithRotation(3.89, -4.04, Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(0)) ;

        // Drive staight along Y axis
        //Pose2dWithRotation dest = new Pose2dWithRotation(0, -4.04, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0)) ;

        // Drive along Y axis, oriented along Y axis
        //Pose2dWithRotation dest = new Pose2dWithRotation(4.04, 0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) ;
        // Same as above but traveling from robot back on inside to robot front inside
        Pose2dWithRotation dest = new Pose2dWithRotation(3.186, 0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) ;



        // Drive
        container_.getDriveTrain().driveTo("test", null, dest, maxv, maxa, 0.0, 0.0, 5.0) ;
    }

    @Override
    public void execute() {
        super.execute() ;

        if (!container_.getDriveTrain().isFollowingPath()) {
            done_ = true ;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
