package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoShootCommand extends Command {
    private CommandSwerveDrivetrain db_ ;
    private IntakeShooterSubsystem intake_shooter_ ;

    public AutoShootCommand(CommandSwerveDrivetrain db, IntakeShooterSubsystem intake_shooter) {
        addRequirements(db, intake_shooter);

        db_ = db ;
        intake_shooter_ = intake_shooter ;
    }
}
