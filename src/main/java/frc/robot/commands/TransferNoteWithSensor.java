package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TransferNoteWithSensor extends Command {
    private enum State {
        MoveToPosition,
        WaitForShooter,
        WaitingForTrampStop,
        Done
    }

    private IntakeShooterSubsystem intake_shooter_;
    private TrampSubsystem tramp_ ;
    private State state_ ;

    public TransferNoteWithSensor(IntakeShooterSubsystem intake_shooter, TrampSubsystem tramp) {
        addRequirements(intake_shooter, tramp);

        intake_shooter_ = intake_shooter;
        tramp_ = tramp ;

        setName("transfer-note-with-sensor") ;
    }

    @Override
    public void initialize() {
        intake_shooter_.abortShot();
        intake_shooter_.moveToTransferPosition();
        tramp_.moveToTransferPosition();
        state_ = State.MoveToPosition ;
    }

    @Override
    public void execute() {
        if (state_ == State.MoveToPosition) {
            if (intake_shooter_.isInTransferPosition() && tramp_.isInTransferPosition()) {
                intake_shooter_.transferWithTrampSensor();
                state_ = State.WaitForShooter ;
            }
        }
        else if (state_ == State.WaitForShooter) {
            if (intake_shooter_.isShooterReady()) {
                tramp_.transferWithTrampSensor();
                state_ = State.WaitingForTrampStop ;
            }            
        }
        else if (state_ == State.WaitingForTrampStop) {
            if (tramp_.isIdle()) {
                intake_shooter_.stopShooterInTransfer() ;
                state_ = State.Done ;
            }
        }

        Logger.recordOutput("xferst", state_) ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}

