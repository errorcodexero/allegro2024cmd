package frc.robot.subsystems.intakeshooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCollectCommand extends Command {
    private IntakeShooterSubsystem intake_shooter_;

    public IntakeCollectCommand(IntakeShooterSubsystem subsystem) {
        addRequirements(subsystem);

        intake_shooter_ = subsystem;
    }

    @Override
    public void initialize() {
        intake_shooter_.collect();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake_shooter_.stopCollect();
        }
    }

    @Override
    public boolean isFinished() {
        return intake_shooter_.isIdle() || intake_shooter_.hasNote() ;
    }
}
