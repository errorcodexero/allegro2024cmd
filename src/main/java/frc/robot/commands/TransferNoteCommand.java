package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TransferNoteCommand extends Command {
    private enum State {
        MoveToPosition,
        TransferringNote,
        MoveTrampToDestination,
        Done
    }

    private IntakeShooterSubsystem intake_shooter_;
    private TrampSubsystem tramp_ ;
    private State state_ ;

    public TransferNoteCommand(IntakeShooterSubsystem intake_shooter, TrampSubsystem tramp) {
        addRequirements(intake_shooter, tramp);

        intake_shooter_ = intake_shooter;
        tramp_ = tramp ;

        setName("transfer-note") ;
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
        switch(state_) {
            case MoveToPosition:
                if (intake_shooter_.isInTransferPosition() && tramp_.isInTransferPosition()) {
                    intake_shooter_.doTransferNote();
                    tramp_.transferNote();
                    state_ = State.TransferringNote ;
                }
                break ;

            case TransferringNote:
                if (intake_shooter_.needStopManipulator())  {
                    tramp_.endNoteTransfer() ;
                    state_ = State.MoveTrampToDestination ;
                }
                break ;

            case MoveTrampToDestination:
                if (tramp_.isIdle()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
