package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CmdTuneRotateDb extends Command {
    private SwerveRotateToAngle rotate_ ;
    private CommandSwerveDrivetrain db_ ;
    private CommandXboxController ctrl_ ;
    private double target_ ;

    public CmdTuneRotateDb(CommandXboxController ctrl, CommandSwerveDrivetrain db) {
        db_ = db ;
        ctrl_ = ctrl ;
        target_ = 0 ;
    }

    @Override
    public void initialize() {
        rotate_ = new SwerveRotateToAngle(db_, () -> { return target_ ; }) ;
        CommandScheduler.getInstance().schedule(rotate_) ;
    }

    @Override
    public void execute() {
        if (rotate_.isFinished()) {
            boolean newcmd = false ;

            if (ctrl_.getHID().getAButtonPressed()) {
                target_ += 45.0 ;
                newcmd = true ;
            }
            else if (ctrl_.getHID().getBButtonPressed()) {
                target_ -= 45.0 ;
                newcmd = true ;
            }
            else if (ctrl_.getHID().getXButtonPressed()) {
                target_ += 5.0 ;
                newcmd = true ;                

            }
            else if (ctrl_.getHID().getYButtonPressed()) {
                target_ -= 5.0 ;
                newcmd = true ;                
            }

            if (newcmd) {
                rotate_ = new SwerveRotateToAngle(db_, () -> { return target_ ; }) ;    
        CommandScheduler.getInstance().schedule(rotate_) ;                            
            }
        }

        Logger.recordOutput("tunedb-target", target_) ;
        Logger.recordOutput("tunedb-current", db_.getState().Pose.getRotation().getDegrees()) ;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false ;
    }
}
