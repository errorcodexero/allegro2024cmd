package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.xero1425.base.XeroRobot;

import edu.wpi.first.wpilibj2.command.Command;

public class ConditionalVibrateCommand extends Command {
    private XeroRobot robot_ ;
    private BooleanSupplier supplier_ ;
    private double duration_ ;

    public ConditionalVibrateCommand(XeroRobot robot, double duration, BooleanSupplier condition) {
        setName("vibrate") ;
        robot_ = robot ;
        duration_ = duration ;
        supplier_ = condition ;
    }

    @Override
    public void initialize() {
        if (supplier_.getAsBoolean()) {
            robot_.setRumble(1.0, duration_) ;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}

