package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TransferNoteCommand extends Command {

    private static final double kTransferDBRampRate = 0.25 ;

    private enum State {
        MoveToPosition,
        TransferringNote,
        WaitForShooterIdle,
        MoveTrampToDestination,
        Done
    }

    private IntakeShooterSubsystem intake_shooter_;
    private TrampSubsystem tramp_ ;
    private CommandSwerveDrivetrain db_ ;
    private State state_ ;

    public TransferNoteCommand(CommandSwerveDrivetrain db, IntakeShooterSubsystem intake_shooter, TrampSubsystem tramp) {
        addRequirements(intake_shooter, tramp);

        intake_shooter_ = intake_shooter;
        tramp_ = tramp ;
        db_ = db ;

        setName("transfer-note") ;
    }

    @Override
    public void initialize() {
        intake_shooter_.setShooterVelocity(0, 0);
        intake_shooter_.moveToTransferPosition();
        tramp_.moveToTransferPosition();
        state_ = State.MoveToPosition ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case MoveToPosition:
                if (intake_shooter_.isInTransferPosition() && tramp_.isInTransferPosition()) {
                    if (db_ != null) {
                        db_.limitDriveMotorRampRate(kTransferDBRampRate) ;
                    }                    
                    intake_shooter_.doTransferNote();
                    tramp_.transferNote();
                    state_ = State.TransferringNote ;
                }
                break ;

            case TransferringNote:
                if (intake_shooter_.needStopManipulator() | tramp_.needStopManipulator()) {
                    tramp_.endNoteTransfer() ;
                    intake_shooter_.endNoteTransfer() ;
                    state_ = State.WaitForShooterIdle ;
                }
                break ;

            case WaitForShooterIdle:
                if (intake_shooter_.finishedShooterOnTransfer()) {
                    tramp_.moveToDestinationPosition();
                    state_ = State.MoveTrampToDestination ;
                }
                break ;

            case MoveTrampToDestination:
                if (tramp_.isInAmpPosition() || tramp_.isInTrapPosition()) {
                    state_ = State.Done ;
                }
                break ;

            case Done:
                break ;
        }

        Logger.recordOutput("states:xfer", state_) ;
    }

    @Override
    public void end(boolean interrupted) {
        state_ = State.Done ;
        if (db_ != null) {
            db_.limitDriveMotorRampRate(0.0) ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }
}
