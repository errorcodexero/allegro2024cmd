package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampTransferCommand extends Command {
    private TrampSubsystem tramp_;

    public TrampTransferCommand(TrampSubsystem subsystem) {
        addRequirements(subsystem);

        tramp_ = subsystem;
    }

    @Override
    public void initialize() {
        tramp_.transfer();
    }

    @Override
    public boolean isFinished() {
        return tramp_.isIdle();
    }    
    
}
