package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampTrapCommand extends Command {
    private TrampSubsystem tramp_ ;

    public TrampTrapCommand(TrampSubsystem sub) {
        tramp_ = sub ;
    }

    @Override
    public void initialize() {
        tramp_.trap() ;
    }

    @Override
    public boolean isFinished() {
        return tramp_.isIdle() ;
    }
}
