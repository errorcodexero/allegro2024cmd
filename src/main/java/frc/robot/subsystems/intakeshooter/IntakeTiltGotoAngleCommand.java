package frc.robot.subsystems.intakeshooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTiltGotoAngleCommand extends Command {
    private IntakeShooterSubsystem sub_ ;
    private double value_ ;
    private double postol_ ;
    private double veltol_ ;

    public IntakeTiltGotoAngleCommand(IntakeShooterSubsystem sub, double value, double postol, double veltol) {
        addRequirements(sub) ;
        value_ = value ;
        sub_ = sub ;
        postol_ = postol ;
        veltol_ = veltol ;
        setName("tilt-goto-angle") ;
    }

    @Override
    public void initialize() {
        sub_.setTiltToAngle(value_, postol_, veltol_) ;
    }

    @Override
    public boolean isFinished() {
        return sub_.isTiltReady() ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
