package frc.robot.commands;

import org.xero1425.base.XeroTimer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class VibrateCommand extends Command {
    private CommandXboxController ctrl_ ;
    private XeroTimer timer_ ;

    public VibrateCommand(CommandXboxController ctrl, double duration) {
        setName("vibrate") ;

        ctrl_ = ctrl ;
        timer_ = new XeroTimer("vibrate-timer", duration) ;
    }

    @Override
    public void initialize() {
        ctrl_.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        timer_.start() ;
    }

    @Override
    public void end(boolean interrupted) {
        ctrl_.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    @Override
    public boolean isFinished() {
        return timer_.isExpired() ;
    }
}

