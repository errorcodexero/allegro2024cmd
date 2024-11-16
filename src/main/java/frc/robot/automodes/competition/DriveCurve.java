package frc.robot.automodes.competition;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AllegroContainer;

public class DriveCurve extends XeroAutoCommand {
    
    private final static String desc = "This auto mode drives straight after a 10 second delay" ;
    private final static double maxv = 1.0 ;
    private final static double maxa = 1.0 ;

    private AllegroContainer container_ ;
    private boolean done_ ;

    private Pose2dWithRotation path[] = {
        new Pose2dWithRotation(0.6, 3.5, Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0)),
        new Pose2dWithRotation(7.75, 1.25, Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0)),
    } ;
    
    public DriveCurve(XeroRobot robot, AllegroContainer container) {
        super(robot, "drive-curve", desc) ;

        container_ = container ;
        addRequirements(container.getDriveTrain());
    }

    @Override
    public void initialize() {
        done_ = false ;
        container_.getDriveTrain().seedFieldRelative(path[0]) ;
        container_.getDriveTrain().driveTo("test", null, path[1], maxv, maxa, 0.0, 0.0, 5.0) ;
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
