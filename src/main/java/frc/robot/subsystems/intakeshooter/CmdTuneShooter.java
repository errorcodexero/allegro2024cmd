package frc.robot.subsystems.intakeshooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;

public class CmdTuneShooter extends Command {

    private enum State {
        WaitingForFirstSettings,
        WaitingForSubsystem,
        WaitingForShot,
        Done,
    }

    private final IntakeShooterSubsystem shooter_;

    private boolean last_apply_value_ = false;

    static private ShuffleboardTab tab_ = null;
    static private SimpleWidget updown_widget_ = null;
    static private SimpleWidget tilt_widget_ = null;
    static private SimpleWidget velocity_widget_ = null;
    static private SimpleWidget apply_widget_ = null;

    static private SuppliedValueWidget<Double> vel1_output_widget_ = null;
    static private SuppliedValueWidget<Double> vel2_output_widget_ = null;
    static private SuppliedValueWidget<Double> updown_output_widget_ = null;
    static private SuppliedValueWidget<Double> tilt_output_widget_ = null;
    static private SuppliedValueWidget<Boolean> updown_ready_widget_ = null;
    static private SuppliedValueWidget<Boolean> tilt_ready_widget_ = null;
    static private SuppliedValueWidget<Boolean> shooter_ready_widget_ = null;
    static private SuppliedValueWidget<String> intake_state_widget_ = null;
    static private SuppliedValueWidget<String> cmd_state_widget_ = null;

    private final double kUpdownPositionTolerance = 1.0;
    private final double kUpdownVelocityTolerance = 1.0;
    private final double kTiltPositionTolerance = 1.0;
    private final double kTiltVelocityTolerance = 1.0;
    private final double kShooterVelocityTolerance = 10.0;

    private State state_;
    private boolean oneshot_ ;

    public CmdTuneShooter(IntakeShooterSubsystem subsystem, boolean oneshot) {
        shooter_ = subsystem;
        oneshot_ = oneshot;
        addRequirements(shooter_);
    }

    @Override
    public void initialize() {
        state_ = State.WaitingForFirstSettings;
        populateShuffleBoard();
        shooter_.startTuning();
    }

    @Override
    public void execute() {
        boolean shouldApplySettings = shouldApplyNewSettings();
        switch (state_) {
            case WaitingForFirstSettings:
                if (shouldApplySettings) {
                    doApplySettings();
                    state_ = State.WaitingForSubsystem;
                }
                break ;

            case WaitingForSubsystem:
                if (shouldApplySettings) {
                    shooter_.stopFeeder();
                    doApplySettings();
                } else if (shooter_.isTiltReady() && shooter_.isUpDownReady() && shooter_.isShooterReady()) {
                    shooter_.startFeeder();
                    state_ = State.WaitingForShot;
                } else {
                    shooter_.stopFeeder();
                    ;
                }
                break;

            case WaitingForShot:
                if (shouldApplySettings) {
                    shooter_.stopFeeder();
                    doApplySettings();
                } else if (shooter_.isTuning()) {
                    //
                    // The shot is done and we are back at tuning again
                    //
                    if (oneshot_) {
                        state_ = State.Done;
                    } else {
                        state_ = State.WaitingForSubsystem;
                    }
                }
                break ;
            
            case Done:
                break ;
        }

        Logger.recordOutput("tune-st", state_.toString());
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done;
    }

    private void doApplySettings() {
        double updown = updown_widget_.getEntry().getDouble(0.0);
        double tilt = tilt_widget_.getEntry().getDouble(0.0);
        double velocity = velocity_widget_.getEntry().getDouble(0.0);

        if (tilt < IntakeShooterConstants.Tilt.kMinPosition) {
            tilt = IntakeShooterConstants.Tilt.kMinPosition;
        } else if (tilt > IntakeShooterConstants.Tilt.kMaxPosition) {
            tilt = IntakeShooterConstants.Tilt.kMaxPosition;
        }

        if (updown < IntakeShooterConstants.UpDown.kMinPosition) {
            updown = IntakeShooterConstants.UpDown.kMinPosition;
        } else if (updown > IntakeShooterConstants.UpDown.kMaxPosition) {
            updown = IntakeShooterConstants.UpDown.kMaxPosition;
        }

        if (velocity < IntakeShooterConstants.Shooter.kShootMinVelocity) {
            velocity = IntakeShooterConstants.Shooter.kShootMinVelocity;
        } else if (velocity > IntakeShooterConstants.Shooter.kShootMaxVelocity) {
            velocity = IntakeShooterConstants.Shooter.kShootMaxVelocity;
        }

        shooter_.setUpDownToAngle(updown, kUpdownPositionTolerance, kUpdownVelocityTolerance);
        shooter_.setTiltToAngle(tilt, kTiltPositionTolerance, kTiltVelocityTolerance);
        shooter_.setShooterVelocity(velocity, kShooterVelocityTolerance);
    }

    private boolean shouldApplyNewSettings() {
        boolean curval = apply_widget_.getEntry().getBoolean(false);
        boolean ret = (curval == true) && (last_apply_value_ == false);

        last_apply_value_ = curval;

        Logger.recordOutput("cmdtune:curval", curval);
        Logger.recordOutput("cmdtune:apply", ret);

        return ret;
    }

    private void populateShuffleBoard() {
        if (tab_ == null) {
            tab_ = Shuffleboard.getTab("TuneShooter");
        }

        if (vel1_output_widget_ == null) {
            vel1_output_widget_ = tab_.addDouble("vel1", () -> {
                return shooter_.getShooter1Velocity();
            }).withSize(1, 1).withPosition(1, 1);
        }

        if (vel2_output_widget_ == null) {
            vel2_output_widget_ = tab_.addDouble("vel2", () -> {
                return shooter_.getShooter2Velocity();
            }).withSize(1, 1).withPosition(2, 1);
        }

        if (tilt_output_widget_ == null) {
            tilt_output_widget_ = tab_.addDouble("tilt", () -> {
                return shooter_.getTilt();
            }).withSize(1, 1).withPosition(1, 2);
        }

        if (updown_output_widget_ == null) {
            updown_output_widget_ = tab_.addDouble("updown", () -> {
                return shooter_.getUpDown();
            }).withSize(1, 1).withPosition(1, 3);
        }

        if (updown_ready_widget_ == null) {
            updown_ready_widget_ = tab_.addBoolean("ud-ready", () -> {
                return shooter_.isUpDownReady();
            }).withSize(1, 1).withPosition(2, 3);
        }

        if (tilt_ready_widget_ == null) {
            tilt_ready_widget_ = tab_.addBoolean("tilt-ready", () -> {
                return shooter_.isTiltReady();
            }).withSize(1, 1).withPosition(2, 2);
        }

        if (shooter_ready_widget_ == null) {
            shooter_ready_widget_ = tab_.addBoolean("shoot-ready", () -> {
                return shooter_.isShooterReady();
            }).withSize(1, 1).withPosition(3, 1);
        }

        if (intake_state_widget_ == null) {
            intake_state_widget_ = tab_.addString("in-state", () -> {
                return shooter_.stateString();
            }).withSize(1, 1).withPosition(1, 0);
        }

        if (cmd_state_widget_ == null) {
            cmd_state_widget_ = tab_.addString("cmd-state", () -> {
                return state_.toString();
            }).withSize(1, 1).withPosition(2, 0);
        }

        if (updown_widget_ == null) {
            updown_widget_ = tab_.add("UpDown Input", IntakeShooterConstants.UpDown.Positions.kShootNominal)
                    .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(0, 3);
        }

        if (tilt_widget_ == null) {
            tilt_widget_ = tab_.add("Tilt Input", -74.0).withWidget(BuiltInWidgets.kTextView).withSize(1, 1)
                    .withPosition(0, 2);
        }

        if (velocity_widget_ == null) {
            velocity_widget_ = tab_.add("Velocity Input", 0.0).withWidget(BuiltInWidgets.kTextView).withSize(1, 1)
                    .withPosition(0, 1);
        }

        if (apply_widget_ == null) {
            apply_widget_ = tab_.add("Apply", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1)
                    .withPosition(0, 0);
        }
    }
}
