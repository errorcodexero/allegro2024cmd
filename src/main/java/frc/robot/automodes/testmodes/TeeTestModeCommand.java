package frc.robot.automodes.testmodes;

import org.xero1425.XeroAutoCommand;

import frc.robot.AllegroContainer;
import frc.robot.AllegroRobot;
import frc.robot.subsystems.intakeshooter.IntakeShooterConstants;

public class TeeTestModeCommand extends XeroAutoCommand {
    AllegroContainer container_;

    public TeeTestModeCommand(AllegroRobot robot, AllegroContainer container) {
        super(robot, "tee-test-mode", "Test Mode moves the updown and tilt to make a tee shape") ;

        container_ = container ;
        addRequirements(container_.getIntakeShooter());
    }

    @Override
    public void initialize() {
        container_.getIntakeShooter().gotoPositionThenIdle(
            90.0, IntakeShooterConstants.UpDown.kTargetPosTolerance, IntakeShooterConstants.UpDown.kTargetVelTolerance,
            0.0, IntakeShooterConstants.Tilt.kTargetPosTolerance, IntakeShooterConstants.Tilt.kTargetVelTolerance) ;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return container_.getIntakeShooter().isIdle() ;
    }
}
