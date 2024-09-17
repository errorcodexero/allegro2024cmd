package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TransferNoteCommand extends Command {
    private enum State {
        MoveToPosition,
        InTransferNoSensor,
        InTransferWithSensor,
        MoveTrampToDestination,
        Done
    }

    private IntakeShooterSubsystem intake_shooter_;
    private TrampSubsystem tramp_ ;
    private State state_ ;
    private boolean use_sensor_ ;

    public TransferNoteCommand(IntakeShooterSubsystem intake_shooter, TrampSubsystem tramp, boolean sensor) {
        addRequirements(intake_shooter, tramp);

        intake_shooter_ = intake_shooter;
        tramp_ = tramp ;
        use_sensor_ = sensor ;

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
        if (state_ == State.MoveToPosition) {
            if (intake_shooter_.isInTransferPosition() && tramp_.isInTransferPosition()) {
                if (use_sensor_) {
                    intake_shooter_.transferWithTrampSensor();
                    tramp_.transferWithTrampSensor();
                }
                else {
                    intake_shooter_.transferNoTrampSensor();
                    tramp_.transferNoTrampSensor(0);
                    state_ = State.InTransferNoSensor ;
                }
            }
        }
        else if (state_ == State.InTransferNoSensor) {
            if (intake_shooter_.needStopManipulator())  {
                tramp_.stopManipulatorHoldNote() ;
                state_ = State.MoveTrampToDestination ;
            }
        }
        else if (state_ == State.InTransferWithSensor) {
            if (tramp_.hasNote()) {
                intake_shooter_.stopShooter() ;
                state_ = State.MoveTrampToDestination ;
            }
        }
        else if (state_ == State.MoveTrampToDestination) {
            if (tramp_.isIdle()) {
                state_ = State.Done ;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
