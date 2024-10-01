package frc.robot.automodes.competition;

import java.util.Optional;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroTimer;
import org.xero1425.paths.XeroPath;
import org.xero1425.paths.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AllegroContainer;

public class DriveStraight extends XeroAutoCommand {

    private final static String desc = "This auto mode drives straight after a 10 second delay" ;
    private static final String kAutoModeNamePathsFile = "DriveStraight" ;
    private static final String kPath1 = "Path1" ;    

    private enum State {
        Delay,
        Drive,
        Done
    } ;

    private XeroTimer delay_ ;
    private XeroPath path1_ ;
    private State state_ ;
    private AllegroContainer container_ ;
    
    public DriveStraight(XeroRobot robot, AllegroContainer container) {
        super(robot, "drive-straight", desc) ;

        container_ = container ;

        delay_ = new XeroTimer("drive-straight", 10.0) ;
        addRequirements(container.getDriveTrain());

        try {
            path1_ = getRobot().getPathManager().getPath(kAutoModeNamePathsFile + "-" + kPath1) ;

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isEmpty())
                throw new Exception("Trying to initialize an automode before alliance is known") ;

            if (alliance.get() == Alliance.Red) {
                double length = robot.getFieldLayout().getFieldLength() ;
                path1_.mirrorX(length) ;
            }
        }
        catch(Exception ex) {
            delay_ = null ;            
        }        
    }

    @Override
    public void initialize() {
        if (delay_ != null) {
            XeroPathSegment seg = path1_.getSegment(0, 0) ;
            container_.getDriveTrain().seedFieldRelative(new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getRotation()))) ;        

            delay_.start() ;
            state_ = State.Delay ;
        }
        else {
            state_ = State.Done ;
        }
    }

    @Override
    public void execute() {
        super.execute() ;

        switch(state_) {
            case Delay:
                if (delay_.isExpired()) {
                    state_ = State.Drive ;
                    container_.getDriveTrain().drivePath(path1_, 0.1) ;
                }
                break ;

            case Drive:
                if (!container_.getDriveTrain().isFollowingPath()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }
    }

    @Override
    public void end(boolean interrupted) {
        container_.getDriveTrain().stopPath();
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
