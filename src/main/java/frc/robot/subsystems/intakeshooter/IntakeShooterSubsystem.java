package frc.robot.subsystems.intakeshooter ;

import org.xero1425.PieceWiseLinear;
import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;
import org.xero1425.XeroTimer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeShooterSubsystem extends XeroSubsystem {

    public enum NoteDestination {
        Speaker,
        Trap,
        Amp
    }
    
    private enum State {
        Idle,
        MoveTiltToPosition,
        MoveBothToPosition,
        WaitForNote,
        WaitForCapture,
        WaitForReverse,
        HoldingTransferPosition,
        WaitingToShoot,
        WaitForShotFinish,
        EjectForward,
        EjectPause,
        EjectReverse,
        TransferStartingShooter,
        TransferWaitForSensor,
        TransferWaitForNoSensor,
        TransferFinishTransfer,
        TransferContinueShooter,
        EnableTracking,
        GoToEjectPosition
    }


    private IntakeShooterIO io_ ;
    private IntakeShooterIOInputsAutoLogged inputs_ ;

    private double target_tilt_ ;
    private double target_tilt_tol_ ;
    private double target_tilt_vel_ ;
    private double next_tilt_ ;
    private double target_updown_ ;
    private double target_updown_tol_ ;
    private double target_updown_vel_ ;
    private double target_velocity_ ;
    private double next_updown_ ;
    private double transfer_start_pos_ ;
    private NoteDestination destination_ ;

    private boolean tracking_ ;
    private DoubleSupplier distsupplier_ ;
    private PieceWiseLinear updown_pwl_ ;
    private PieceWiseLinear tilt_pwl_ ;
    private PieceWiseLinear velocity_pwl_ ;

    private boolean has_note_ ;
    private boolean need_stop_manipulator_ ;

    private XeroTimer capture_timer_ ;
    private XeroTimer reverse_timer_ ;
    private XeroTimer shoot_timer_ ;
    private XeroTimer eject_forward_timer_ ;
    private XeroTimer eject_reverse_timer_ ;
    private XeroTimer eject_pause_timer_ ;
    private XeroTimer transfer_shooter_to_feeder_timer_ ;

    private State state_ ;
    private State next_state_ ;
    private State initial_transfer_state_ ;

    private Command eject_command_ ;
    private Command collect_command_ ;
    private Command turtle_command_ ;

    private Trigger transfer_note_trigger_ ;

    public IntakeShooterSubsystem(XeroRobot robot, DoubleSupplier distsupplier) throws Exception {
        super(robot, "intake-shooter") ;

        io_ = new IntakeShooterIOTalonFX(getRobot().isPracticeBot()) ;
        inputs_ = new IntakeShooterIOInputsAutoLogged() ;

        capture_timer_ = new XeroTimer(getRobot(), "collect-timer", IntakeShooterConstants.kCollectDelayTime) ;
        reverse_timer_ = new XeroTimer(getRobot(), "reverse-timer", IntakeShooterConstants.kReverseDelayTime) ;
        shoot_timer_ = new XeroTimer(getRobot(), "shoot-timer", IntakeShooterConstants.Feeder.kShootTime) ;
        eject_forward_timer_ = new XeroTimer(getRobot(), "eject-1", IntakeShooterConstants.Shooter.kEjectForwardTime) ;
        eject_reverse_timer_ = new XeroTimer(getRobot(), "eject-1", IntakeShooterConstants.Shooter.kEjectReverseTime) ;
        eject_pause_timer_ = new XeroTimer(getRobot(), "eject-1", IntakeShooterConstants.Shooter.kEjectPauseTime) ;
        transfer_shooter_to_feeder_timer_ = new XeroTimer(getRobot(), "transfer-shooter-to-feeder", IntakeShooterConstants.kTransferFeederToShooterDelay) ;

        io_.setTiltMotorPosition(io_.getTiltAbsoluteEncoderPosition());
        io_.setUpDownMotorPosition(IntakeShooterConstants.UpDown.Positions.kStowed);

        tracking_ = false ;
        distsupplier_ = distsupplier ;
        updown_pwl_ = new PieceWiseLinear(IntakeShooterConstants.UpDown.kPwlValues) ;
        tilt_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Tilt.kPwlValues) ;
        velocity_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Shooter.kPwlValues) ;

        state_ = State.Idle ;
        destination_ = NoteDestination.Speaker ;

        transfer_note_trigger_ = new Trigger(this::transferNote) ;

        eject_command_ = new IntakeEjectCommand(this) ;
        collect_command_ = new IntakeCollectCommand(this) ;
        turtle_command_ = new IntakeTurtleCommand(this) ;

        need_stop_manipulator_ = false ;
    }

    public Trigger getTransferNoteTrigger() {
        return transfer_note_trigger_ ;
    }

    public boolean hasNote() {
        return has_note_ ;
    }

    public boolean transferNote() {
        return has_note_ && (destination_ == NoteDestination.Trap ||  destination_ == NoteDestination.Amp) ;
    }

    public boolean needStopManipulator() {
        return need_stop_manipulator_ ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Subsystem command factor interface
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    public Command collectCommand() {
        return collect_command_ ;
    }

    public Command ejectCommand() {
        return eject_command_ ;
    }

    public Command turtleCommand() {
        return turtle_command_ ;
    }

    public Command targetSpeakerCommand() {
        return runOnce(() -> destination_ = NoteDestination.Speaker) ;
    }

    public Command targetTrapCommand() {
        return runOnce(() -> destination_ = NoteDestination.Trap) ;
    }    

    public Command targetAmpCommand() {
        return runOnce(() -> destination_ = NoteDestination.Amp) ;
    }

    public Command shootCommand() {
        return runOnce(this::finishShot) ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // State about the subsystem
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public boolean isNoteDetected() {
        return inputs_.noteSensor ^ IntakeShooterConstants.NoteSensor.kInverted ;
    }

    public boolean isIdle() {
        return state_ == State.Idle ;
    }

    public boolean isInTransferPosition() {
        return state_ == State.HoldingTransferPosition ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Methods that cause the subsystem to take action
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //
    // If the intake is idle, and does not have a note, this method collects a new note.
    //
    public void collect() {
        if (!has_note_ && state_ == State.Idle) {
            //
            // Turn on the feeder
            //
            io_.setFeederVoltage(IntakeShooterConstants.Feeder.kCollectVoltage);

            //
            // Move the updown and tilt to the collect position
            //
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kCollect, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kCollect, Double.NaN, Double.NaN) ;
            
            //
            // And set the state for after we reach the collect position
            //
            next_state_ = State.WaitForNote ;                         
        }
    }

    //
    // If we are collecting, this stops the collect operation.
    //
    public void stopCollect() {
        if (isMoving() || state_ == State.WaitForNote) {
            //
            // Turn off the feeder and stop the tilt and updown
            //
            io_.setFeederVoltage(0.0) ;
            io_.setTiltTargetPos(inputs_.tiltPosition);
            io_.setUpDownMotorPosition(inputs_.updownPosition);
            reverse_timer_.start() ;

            state_ = State.WaitForReverse ;
        }
    }

    //
    // If the intake has a note, this method transfers the note to the manipulator.  This
    // method assumes the intake/shooter and elevator/manipulator are in the correct positions
    // before being called.  If they are in the transfer position, the state will be HoldingTransferPosition.
    //
    public void transfer() {
        if (has_note_ && state_ == State.HoldingTransferPosition) {
            need_stop_manipulator_ = false ;

            if (inputs_.noteSensor) {
                //
                // The note is already sitting on the sensor.  We just wait for the note to
                // move off the sensor
                //
                initial_transfer_state_ = State.TransferWaitForNoSensor ;
            }
            else {
                //
                // The note is not on the sensor, so the sensor is assumed to be sitting in the middle of the note.
                // We wait until we sense the note and then look for the note to move off the sensor.
                //
                initial_transfer_state_ = State.TransferWaitForSensor ;
            }

            setShooterVelocity(IntakeShooterConstants.kTransferShooterVelocity);
            transfer_shooter_to_feeder_timer_.start() ;
            state_ = State.TransferStartingShooter ;            
        }
    }

    //
    // If we are in the idle state, this method moves the mechanisms to the stowed position.
    //
    public void turtle() {
        if (state_ == State.Idle) {
            //
            // Move the updown and tilt to the collect position
            //
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kStowed, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kStowed, Double.NaN, Double.NaN) ;
            
            //
            // And set the state for after we reach the collect position
            //
            next_state_ = State.Idle ;
        }
    }

    //
    // This method moves the intake/shooter and the elevator/arm to the transfer positions.
    //
    public void moveToTransferPosition() {
        if (has_note_ && state_ == State.Idle) {
            //
            // Move the updown and tilt to the transfer position
            //
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kTransfer, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kTransfer, Double.NaN, Double.NaN) ;
            
            //
            // And set the state for after we reach the transfer position
            //
            next_state_ = State.HoldingTransferPosition ;                         
        }
    }

    public void manualShoot(double updown, double tilt, double velocity) {
        gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;
        setShooterVelocity(velocity);
        state_ = State.WaitingToShoot ;
    }

    public void finishShot() {
        //
        // This locks in the parametesr for the shot.
        //
        tracking_ = false ;

        state_ = State.WaitingToShoot ;
    }

    public void eject() {
        gotoPosition(IntakeShooterConstants.UpDown.Positions.kEject, Double.NaN, Double.NaN, 
                     IntakeShooterConstants.Tilt.Positions.kEject, Double.NaN, Double.NaN) ;
        state_ = State.GoToEjectPosition ;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Implementation
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void setUpDownTarget(double pos) {
        io_.setUpDownTargetPos(pos);
        target_updown_ = pos ;

    }

    public void setTiltTarget(double pos) {
        io_.setTiltTargetPos(pos);
        target_tilt_ = pos ;
    }

    private void setShooterVelocity(double vel) {
        io_.setShooter1Velocity(vel);
        io_.setShooter2Velocity(vel);
        target_velocity_ = vel ;
    }

    private void setShooterVoltage(double v) {
        io_.setShooter1Voltage(v);
        io_.setShooter2Voltage(v);
    }

    private boolean isMoving() {
        return state_ == State.MoveTiltToPosition || state_ == State.MoveBothToPosition ;
    }

    //
    // These methods corespond to the the states the subsystem can be in
    //
    private void moveTiltToPositionState() {
        if (isTiltReady()) {
            setUpDownTarget(next_updown_);
            setTiltTarget(next_tilt_);
            state_ = State.MoveBothToPosition ;
        }
    }

    //
    // Wait for the intake and tilt to move to the target position and when there, go to the state given
    // by next_state_ ;
    //
    private void moveBothToPositionState() {
        if (isTiltReady() && isUpDownReady()) {
            state_ = next_state_ ;
        }
    }

    private void waitForNoteState() {
        if (inputs_.risingEdge || inputs_.fallingEdge) {
            //
            // Set the note flag, start moving the intake to the target position, and wait for the feeder to run
            // until we have the note fully captured.
            //
            has_note_ = true ;
            capture_timer_.start() ;
            state_ = State.WaitForCapture ;
        }
    }

    private void waitForCaptureState() {
        if (capture_timer_.isExpired()) {
            //
            // Turn off the feeder
            //
            io_.setFeederVoltage(0.0);

            //
            // Move the updown and tilt to the stowed position
            //
            double updown ;
            double tilt ;
            
            if (has_note_) {
                switch(destination_) {
                    case Speaker:
                        updown = IntakeShooterConstants.UpDown.Positions.kStartTracking ;
                        tilt = IntakeShooterConstants.Tilt.Positions.kStartTracking;                    
                        next_state_ = State.EnableTracking ;
                        break ;

                    case Trap:
                    case Amp:
                        updown = IntakeShooterConstants.UpDown.Positions.kTransfer ;
                        tilt = IntakeShooterConstants.Tilt.Positions.kTransfer ;
                        next_state_ = State.Idle ;                        
                        break ;

                    default:
                        updown = IntakeShooterConstants.UpDown.Positions.kStowed ;
                        tilt = IntakeShooterConstants.Tilt.Positions.kStowed ;
                        next_state_ = State.Idle ;
                        break ;
                }
                gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;
            }
        }
    }

    private void waitForReverseState() {
        if (reverse_timer_.isExpired()) {
            //
            // Move the updown and tilt to the stowed position
            //
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kStowed, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kStowed, Double.NaN, Double.NaN) ;

            //
            // And set the state for after we reach the stowed position
            //
            next_state_ = State.Idle ;
        }
    }

    private boolean isTiltReady() {
        return 
            Math.abs(inputs_.tiltPosition - target_tilt_) < target_tilt_tol_ &&
            Math.abs(inputs_.tiltVelocity) < target_tilt_vel_ ;
    }

    private boolean isUpDownReady() {
        return 
            Math.abs(inputs_.updownPosition - target_updown_) < target_updown_tol_ &&
            Math.abs(inputs_.updownVelocity) < target_updown_vel_ ;
    }

    private boolean isShooterReady() {
        return 
            Math.abs(inputs_.shooter1Velocity - target_velocity_) < IntakeShooterConstants.Shooter.kShooterVelocityTolerance &&
            Math.abs(inputs_.shooter2Velocity - target_velocity_) < IntakeShooterConstants.Shooter.kShooterVelocityTolerance ;
    }

    private void waitingToShootState() {
        if (isTiltReady() && isUpDownReady() && isShooterReady()) {
            io_.setFeederVoltage(IntakeShooterConstants.Feeder.kShootVoltage) ;
            shoot_timer_.start() ;
            state_ = State.WaitForShotFinish ;
        }
    }

    private void waitForShotFinishState() {
        if (shoot_timer_.isExpired()) {
            //
            // Turn off the feeder
            //
            io_.setFeederVoltage(0.0);

            //
            // Move the updown and tilt to the stowed position
            //
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kStowed, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kStowed, Double.NaN, Double.NaN) ;

            //
            // And set the state for after we reach the stowed position
            //
            next_state_ = State.Idle ;
        }
    }

    private void ejectForwardState() {
        if (eject_forward_timer_.isExpired()) {
            setShooterVoltage(0.0);
            io_.setFeederVoltage(0.0);
            eject_pause_timer_.start() ;
            state_ = State.EjectPause ;
        }
    }

    private void ejectPauseState() {
        if (eject_pause_timer_.isExpired()) {
            setShooterVoltage(-IntakeShooterConstants.Shooter.kEjectVoltage) ;
            io_.setFeederVoltage(-IntakeShooterConstants.Feeder.kEjectVoltage) ;
            eject_reverse_timer_.start() ;
            state_ = State.EjectReverse ;
        }
    }

    private void ejectReverseState() {
        if (eject_reverse_timer_.isExpired()) {
            setShooterVoltage(0.0);
            io_.setFeederVoltage(0.0);
            state_ = State.Idle ;
        }
    }

    private void transferStartedFeederState() {
        if (transfer_shooter_to_feeder_timer_.isExpired()) {
            io_.setFeederVoltage(IntakeShooterConstants.Feeder.kTransferVoltage);
            state_ = initial_transfer_state_ ;
        }
    }

    private void trackTargetDistance() {
        double dist = distsupplier_.getAsDouble() ;
        double updown = updown_pwl_.getValue(dist) ;
        double tilt = tilt_pwl_.getValue(dist) ;
        double velocity = velocity_pwl_.getValue(dist) ;

        if (Math.abs(updown - target_updown_) > 5.0) {
            setUpDownTarget(updown);
        }

        setTiltTarget(tilt) ;
        setShooterVelocity(velocity) ;
    }

    @Override
    public void simulationPeriodic() {
        io_.simulate(getRobot().getPeriod()) ;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);

        Logger.processInputs("intake-shooter", inputs_);

        switch(state_) {
            case Idle:
                break ;

            case MoveTiltToPosition:
                moveTiltToPositionState() ;
                break ;

            case MoveBothToPosition:
                moveBothToPositionState() ;
                break ;

            case WaitForNote:
                waitForNoteState() ;
                break ;

            case WaitForCapture:
                waitForCaptureState() ;
                break ;

            case WaitForReverse:
                waitForReverseState() ;
                break ;

            case HoldingTransferPosition:
                break ;

            case WaitingToShoot:
                waitingToShootState() ;
                break ;

            case WaitForShotFinish:
                waitForShotFinishState() ;
                break ;

            case EjectForward:
                ejectForwardState() ;
                break ;

            case EjectPause:
                ejectPauseState() ;
                break ;

            case EjectReverse:
                ejectReverseState() ;
                break ;

            case TransferStartingShooter:
                transferStartedFeederState() ;
                break ;

            case TransferWaitForSensor:
                if (isNoteDetected()) {
                    state_ = State.TransferWaitForNoSensor ;
                }
                break ;

            case TransferWaitForNoSensor:
                if (!isNoteDetected()) {
                    state_ = State.TransferFinishTransfer ;
                    transfer_start_pos_ = inputs_.shooter1Position ;
                }
                break ;

            case TransferFinishTransfer:
                if (inputs_.shooter1Position - transfer_start_pos_ > IntakeShooterConstants.kTransferShooterTransferLength) {
                    need_stop_manipulator_ = true ;
                    state_ = State.TransferContinueShooter ;
                }
                break ;

            case TransferContinueShooter:
                if (inputs_.shooter1Position - transfer_start_pos_ - IntakeShooterConstants.kTransferShooterTransferLength > IntakeShooterConstants.kTransferShooterContLength) {
                    io_.setFeederVoltage(0.0);
                    setShooterVoltage(0.0);
                    state_ = State.Idle ;
                }
                break ;

            case EnableTracking:
                tracking_ = true ;
                state_ = State.Idle ;
                break ;

            case GoToEjectPosition:
                if (isTiltReady() && isUpDownReady()) {
                    setShooterVoltage(IntakeShooterConstants.Shooter.kEjectVoltage);
                    eject_forward_timer_.start() ;
                    has_note_ = false ;     
                    state_ = State.EjectForward ;
                }
                break ;
        }

        if (tracking_ && distsupplier_ != null) {
            trackTargetDistance() ;
        }

        Logger.recordOutput("intake-shooter-state", state_);        
        Logger.recordOutput("updown-target", target_updown_) ;
        Logger.recordOutput("tilt-target", target_tilt_) ;
        Logger.recordOutput("shooter-velocity", target_velocity_) ;
        Logger.recordOutput("is-tilt-ready", isTiltReady());
        Logger.recordOutput("is-updown-ready", isUpDownReady());
        Logger.recordOutput("has-note", has_note_);
        Logger.recordOutput("tracking", tracking_);
    }

    private double computeTiltFromUpdown(double updown) {
        double updown_range = IntakeShooterConstants.UpDown.Positions.kStowed - IntakeShooterConstants.UpDown.Positions.kCollect ;
        double tilt_range = IntakeShooterConstants.Tilt.Positions.kStowed - IntakeShooterConstants.Tilt.Positions.kCollect ;
        double pcnt = (IntakeShooterConstants.UpDown.Positions.kStowed - updown) / updown_range ;
        double ret = IntakeShooterConstants.Tilt.Positions.kStowed - pcnt * tilt_range ;

        if (ret < IntakeShooterConstants.Tilt.Positions.kStowed) {
            ret = IntakeShooterConstants.Tilt.Positions.kStowed ;
        }
        else if (ret > IntakeShooterConstants.Tilt.Positions.kCollect) {
            ret = IntakeShooterConstants.Tilt.Positions.kCollect ;
        }
        else if (updown < 15.0 && ret < IntakeShooterConstants.Tilt.Positions.kCollect) {
            //
            // So as the updown has the tilt below the bumpers, we don't try to pull back into the
            // bumpers.
            //
            ret = 50.0 ;
        }

        return ret;
    }

    private void gotoPosition(double updown, double updowntol, double updownvel, double tilt, double tilttol, double tiltvel) {

        next_tilt_ = tilt ;
        next_updown_ = updown ;

        if (Double.isNaN(updowntol))
            target_updown_tol_ = IntakeShooterConstants.UpDown.kTargetPosTolerance ;
        else
            target_updown_tol_ = updowntol ;

        if (Double.isNaN(updownvel))
            target_updown_vel_ = IntakeShooterConstants.UpDown.kTargetVelTolerance ;
        else
            target_updown_vel_ = updownvel ;

        if (Double.isNaN(tilttol))
            target_tilt_tol_ = IntakeShooterConstants.Tilt.kTargetPosTolerance ;
        else
            target_tilt_tol_ = tilttol ;

        if (Double.isNaN(tiltvel))
            target_tilt_vel_ = IntakeShooterConstants.Tilt.kTargetVelTolerance ;
        else
            target_tilt_vel_ = tiltvel ;

        //
        // Compute the tilt position that would be required based on where the current updown
        // is located to be on the "standard" path.  This is used to determine if we need to move the
        // tilt alone first, or can move the updown and tilt together.
        //
        double desired_tilt = computeTiltFromUpdown(inputs_.updownPosition) ;
        if (Math.abs(inputs_.tiltPosition - desired_tilt) > IntakeShooterConstants.Tilt.kAllowedDeviationFromTrack) {
            //
            // First align tilt to the desired position, then move them to the desired position together
            //
            setTiltTarget(desired_tilt) ;
            state_ = State.MoveTiltToPosition ;            
        }
        else {
            //
            // Single move of updown/tilt together
            //
            setUpDownTarget(updown);
            setTiltTarget(tilt);
            state_ = State.MoveBothToPosition ;            
        }
    }
}
