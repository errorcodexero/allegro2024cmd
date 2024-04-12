package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampTurtleCommand extends Command {
    private TrampSubsystem tramp_;

    public TrampTurtleCommand(TrampSubsystem subsystem) {
        addRequirements(subsystem);

        tramp_ = subsystem;
    }

    @Override
    public void initialize() {
        tramp_.turtle();
    }

    @Override
    public boolean isFinished() {
        return tramp_.isIdle();
    }        
}
