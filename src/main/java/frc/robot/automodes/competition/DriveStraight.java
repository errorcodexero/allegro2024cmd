package frc.robot.automodes.competition;

import org.littletonrobotics.junction.Logger;
import org.xero1425.Pose2dWithRotation;
import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AllegroContainer;

public class DriveStraight extends XeroAutoCommand {

    private enum State {
        Start,
        Driving1,
        Delaying,
        Driving2,
        Done,
    } ;

    private State state_ ;
    private AllegroContainer container_ ;
    private double timer_ ;
    private double velocity_ = 1.0 ;
    private double accel_ = 1.0 ;
    private double dist_ = 2.00 ;

    // 394 cm

    private final static String desc = "This auto mode tests a drive straight" ;
    
    public DriveStraight(XeroRobot robot, AllegroContainer container) {
        super(robot, "drive-straight", desc) ;

        container_ = container ;
        state_ = State.Start ;

        addRequirements(container.getDriveTrain(), container.getIntakeShooter(), container.getTramp());
    }

    @Override
    public void initialize() {
        Pose2dWithRotation target = new Pose2dWithRotation(dist_, 0.0, new Rotation2d(), new Rotation2d()) ;
        container_.getDriveTrain().seedFieldRelative(new Pose2d()) ;
        container_.getDriveTrain().driveTo("Straight", null, target, 
                    velocity_, accel_, 0, 0, 5.0) ;

        state_ = State.Driving1 ;
    }

    @Override
    public void execute() {
        super.execute() ;

        switch(state_) {
            case Start:
                break ;

            case Driving1:
                if (!container_.getDriveTrain().isFollowingPath())
                {
                    state_ = State.Done ;
                    timer_ = Timer.getFPGATimestamp() ;
                }
                break ;

            case Delaying:
                if (Timer.getFPGATimestamp() - timer_ > 2.0)
                {
                    Pose2dWithRotation target = new Pose2dWithRotation(0.0, 0.0, new Rotation2d(), new Rotation2d()) ;
                    container_.getDriveTrain().driveTo("Straight", null, target, 
                            velocity_, accel_, 0, 0, 5.0) ;                    
                            state_ = State.Driving2 ;
                }
                break ;

            case Driving2:
                if (!container_.getDriveTrain().isFollowingPath())
                {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("drive-straight", state_) ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
