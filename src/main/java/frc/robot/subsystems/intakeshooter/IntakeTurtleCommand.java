package frc.robot.subsystems.intakeshooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTurtleCommand extends Command {
    private IntakeShooterSubsystem intake_shooter_;

    public IntakeTurtleCommand(IntakeShooterSubsystem subsystem) {
        addRequirements(subsystem);

        intake_shooter_ = subsystem;
    }

    @Override
    public void initialize() {
        intake_shooter_.turtle();
    }

    @Override
    public boolean isFinished() {
        return intake_shooter_.isIdle() ;
    }    
}
