package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class StopCollectCommand extends Command {
    private IntakeShooterSubsystem intake_shooter_ ;
    private boolean is_finished_ ;
    
    public StopCollectCommand(IntakeShooterSubsystem intake_shooter) {
        intake_shooter_ = intake_shooter ;
        addRequirements(intake_shooter_) ;
    }

    @Override
    public void initialize() {
        is_finished_ = false ;
        intake_shooter_.stopCollect();
    }

    @Override
    public void execute() {
        if (intake_shooter_.isIdle()) {
            is_finished_ = true ;
        }
    }

    @Override
    public boolean isFinished() {
        return is_finished_ ;
    }
}
