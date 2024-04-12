package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampTransferPositionCommand extends Command {
    private TrampSubsystem tramp_;

    public TrampTransferPositionCommand(TrampSubsystem subsystem) {
        addRequirements(subsystem);

        tramp_ = subsystem;
    }

    @Override
    public void initialize() {
        tramp_.moveToTransferPosition();
    }

    @Override
    public boolean isFinished() {
        return tramp_.isIdle();
    }        
}
