package frc.robot.subsystems.intakeshooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeMoveToTransferPositionCommand extends Command {
    private IntakeShooterSubsystem intake_shooter_;

    public IntakeMoveToTransferPositionCommand(IntakeShooterSubsystem subsystem) {
        addRequirements(subsystem);

        intake_shooter_ = subsystem;
    }

    @Override
    public void initialize() {
        intake_shooter_.moveToTransferPosition();
    }

    @Override
    public boolean isFinished() {
        return intake_shooter_.isInTransferPosition() ;
    }    
}
