package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampShootCommand extends Command {
    private TrampSubsystem tramp_ ;

    public TrampShootCommand(TrampSubsystem sub) {
        tramp_ = sub ;
    }

    @Override
    public void initialize() {
        tramp_.shoot() ;
    }

    @Override
    public boolean isFinished() {
        return tramp_.isIdle() ;
    }

}
