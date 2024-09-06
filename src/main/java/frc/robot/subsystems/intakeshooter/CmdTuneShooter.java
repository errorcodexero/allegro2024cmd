package frc.robot.subsystems.intakeshooter;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;

public class CmdTuneShooter extends Command {
    private final IntakeShooterSubsystem shooter_;
    private ShuffleboardTab tab_ ;
    private SimpleWidget updown_widget_ ;
    private SimpleWidget tilt_widget_ ;
    private SimpleWidget velocity_widget_ ;
    private SimpleWidget apply_widget_ ;
    private boolean last_apply_value_  = false ;

    private final double kUpdownPositionTolerance = 1.0 ;
    private final double kUpdownVelocityTolerance = 1.0 ;    
    private final double kTiltPositionTolerance = 1.0 ;
    private final double kTiltVelocityTolerance = 1.0 ;
    private final double kShooterVelocityTolerance = 10.0 ;

    public CmdTuneShooter(IntakeShooterSubsystem subsystem) {
        shooter_ = subsystem;
        addRequirements(shooter_);
    }
    
    @Override
    public void initialize() {
        if (tab_ == null) {
            tab_ = Shuffleboard.getTab("TuneShooter") ;
            tab_.addDouble("vel1", ()-> { return shooter_.getShooter1Velocity();}) ;
            tab_.addDouble("vel2", ()-> { return shooter_.getShooter2Velocity();}) ;
            tab_.addDouble("tilt", ()-> { return shooter_.getTilt();}) ;
            tab_.addDouble("updown", ()-> { return shooter_.getUpDown();}) ;

            tab_.addBoolean("ud-ready", ()-> { return shooter_.isUpDownReady();}) ; 
            tab_.addBoolean("tilt-ready", ()-> { return shooter_.isTiltReady();}) ;
            tab_.addBoolean("shoot-ready", ()-> { return shooter_.isShooterReady();}) ;
        }

        updown_widget_ = tab_.add("UpDown Input", IntakeShooterConstants.UpDown.Positions.kShootNominal).withWidget(BuiltInWidgets.kTextView) ;
        tilt_widget_ = tab_.add("Tilt Input", -74.0).withWidget(BuiltInWidgets.kTextView) ;
        velocity_widget_ = tab_.add("Velocity Input", 0.0).withWidget(BuiltInWidgets.kTextView) ;
        apply_widget_ = tab_.add("Apply", false).withWidget(BuiltInWidgets.kToggleSwitch) ;
    }

    @Override
    public void execute() {
        if (applySettings()) {
            apply_widget_.getEntry().setBoolean(false) ;

            double updown = updown_widget_.getEntry().getDouble(0.0) ;
            double tilt = tilt_widget_.getEntry().getDouble(0.0) ;
            double velocity = velocity_widget_.getEntry().getDouble(0.0) ;

            if (tilt < IntakeShooterConstants.Tilt.kMinPosition) {
                tilt = IntakeShooterConstants.Tilt.kMinPosition ;
            }
            else if (tilt > IntakeShooterConstants.Tilt.kMaxPosition) {
                tilt = IntakeShooterConstants.Tilt.kMaxPosition ;
            }

            if (updown < IntakeShooterConstants.UpDown.kMinPosition) {
                updown = IntakeShooterConstants.UpDown.kMinPosition ;
            }
            else if (updown > IntakeShooterConstants.UpDown.kMaxPosition) {
                updown = IntakeShooterConstants.UpDown.kMaxPosition ;
            }
            
            if (velocity < IntakeShooterConstants.Shooter.kShootMinVelocity) {
                velocity = IntakeShooterConstants.Shooter.kShootMinVelocity ;
            }
            else if (velocity > IntakeShooterConstants.Shooter.kShootMaxVelocity) {
                velocity = IntakeShooterConstants.Shooter.kShootMaxVelocity ;
            }

            shooter_.setUpDownToAngle(updown, kUpdownPositionTolerance, kUpdownVelocityTolerance) ;
            shooter_.setTiltToAngle(tilt, kTiltPositionTolerance, kTiltVelocityTolerance) ;
            shooter_.setShooterVelocity(velocity, kShooterVelocityTolerance) ;
        }

        if (shooter_.isIdle()) {
            shooter_.shoot() ;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean applySettings() {
        boolean curval = apply_widget_.getEntry().getBoolean(false) ;
        boolean ret = curval && !last_apply_value_ ;

        last_apply_value_ = curval ;
        return ret ;        
    }
}
