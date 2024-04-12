package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampEjectCommand extends Command {
    private TrampSubsystem tramp_;

    public TrampEjectCommand(TrampSubsystem subsystem) {
        addRequirements(subsystem);

        tramp_ = subsystem;
    }

    @Override
    public void initialize() {
        tramp_.eject();
    }

    @Override
    public boolean isFinished() {
        return tramp_.isIdle();
    }    
}
