package frc.robot.automodes.competition;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.AllegroContainer;
import frc.robot.DrivePathDetectNoteCmd;

public class DrivePathDetectNote extends XeroAutoCommand {

    private DrivePathDetectNoteCmd cmd_ ;

    private Pose2dWithRotation path[] = {
        new Pose2dWithRotation(0.75, 4.44, Rotation2d.fromDegrees(-60.0), Rotation2d.fromDegrees(-60.0)),
        new Pose2dWithRotation(4.0, 1.15, Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0)),
        new Pose2dWithRotation(8.0, 0.75, Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0))
    } ;
    
    private final static String desc = "This auto mode drives straight after a 10 second delay" ;
    private final static double maxv = 1.0 ;
    private final static double maxa = 1.0 ;

    private AllegroContainer container_ ;
    private boolean done_ ;
    
    public DrivePathDetectNote(XeroRobot robot, AllegroContainer container) {
        super(robot, "drive-straight", desc) ;

        container_ = container ;
        addRequirements(container.getDriveTrain());
    }

    @Override
    public void initialize() {
        done_ = false ;
        container_.getDriveTrain().seedFieldRelative(path[0]) ;
        Translation2d note = new Translation2d(8.25, 0.8) ;
        cmd_ = new DrivePathDetectNoteCmd("detect", container_.getDriveTrain(), path, note, maxv, maxa, 2.0) ;
    }

    @Override
    public void execute() {
        super.execute() ;
        if (cmd_.isFinished()) {
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
