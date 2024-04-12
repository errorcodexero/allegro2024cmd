package frc.robot.subsystems.intakeshooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTransferCommand extends Command {
    private IntakeShooterSubsystem intake_shooter_;

    public IntakeTransferCommand(IntakeShooterSubsystem subsystem) {
        addRequirements(subsystem);

        intake_shooter_ = subsystem;
    }

    @Override
    public void initialize() {
        intake_shooter_.transfer();
    }

    @Override
    public boolean isFinished() {
        return intake_shooter_.isIdle() ;
    }      
}
