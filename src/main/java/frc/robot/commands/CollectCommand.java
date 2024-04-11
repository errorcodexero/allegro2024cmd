package frc.robot.commands;

import org.xero1425.XeroRobot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AllegroContainer;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class CollectCommand extends Command {
    private IntakeShooterSubsystem intake_shooter_ ;
    private boolean is_finished_ ;

    public CollectCommand(XeroRobot robot) {
        AllegroContainer container = (AllegroContainer)robot.getContainer() ;
        intake_shooter_ = container.getIntakeShooter() ;
        addRequirements(intake_shooter_) ;
    }

    @Override
    public void initialize() {
        super.initialize() ;
        is_finished_ = false ;
        intake_shooter_.collect() ;
    }

    @Override
    public void execute() {
        super.execute() ;

        if (intake_shooter_.isIdle()) {
            is_finished_ = true ;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted) ;
        intake_shooter_.stopCollect() ;
    }

    @Override
    public boolean isFinished() {
        return is_finished_ ;}
}
