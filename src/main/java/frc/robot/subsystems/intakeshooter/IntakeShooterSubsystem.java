package frc.robot.subsystems.intakeshooter ;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.base.XeroTimer;
import org.xero1425.math.PieceWiseLinear;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units ;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AllegroContainer;
import frc.robot.NoteDestination;
import frc.robot.ShotType;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.OISubsystem.OILed;

public class IntakeShooterSubsystem extends XeroSubsystem {

    // #region constant strings, used for simulator
    static final public String NAME = "IntakeShooterSubsystem" ;
    static final public String FEEDER_MOTOR_NAME = "feeder" ;
    static final public String SHOOTER1_MOTOR_NAME = "shooter1" ;
    static final public String SHOOTER2_MOTOR_NAME = "shooter2" ;
    static final public String UPDOWN_MOTOR_NAME = "updown" ;
    static final public String TILT_MOTOR_NAME = "tilt" ;        
    // #endregion
   
    // #region state machine states for controlling the intake/shooter
    private enum State {
        Invalid,
        Idle,

        MoveTiltToPosition,
        MoveBothToPosition,

        WaitForNote,
        WaitForCapture,

        WaitForReverse,
        HoldingTransferPosition,
        WaitingToShoot,
        WaitForShotFinish,

        GoToEjectPosition,
        EjectForward,
        EjectPause,
        EjectReverse,

        HoldForShoot,

        Tuning,

        TransferWaitForNote,
        TransferWaitForNoNoteCenter,
        TransferWaitForNoNote2,
        TransferRunToManipulatorStop,
        TransferRunToShooterStop,

        WaitingForTunedNoteShot1,
        WaitingForTunedNoteShot2,        
    }
    // #endregion

    // #region private member variables
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
    private double target_velocity_tol_ ;
    private double next_updown_ ;
    private double transfer_start_pos_ ;

    private double auto_manual_shoot_updown_ ;
    private double auto_manual_shoot_tilt_ ;

    private boolean tracking_ ;
    private DoubleSupplier distsupplier_ ;
    private PieceWiseLinear updown_pwl_ ;
    private PieceWiseLinear tilt_pwl_ ;
    private PieceWiseLinear velocity_pwl_ ;
    private LinearFilter average_filter_ ;
    private double average_value_ ;
    private double last_value_ ;
    private double last_time_ ;

    private boolean has_note_ ;
    private boolean need_stop_manipulator_ ;

    private XeroTimer capture_timer_ ;
    private XeroTimer reverse_timer_ ;
    private XeroTimer shoot_timer_ ;
    private XeroTimer eject_forward_timer_ ;
    private XeroTimer eject_reverse_timer_ ;
    private XeroTimer eject_pause_timer_ ;

    private State state_ ;
    private State next_state_ ;
    private boolean auto_mode_auto_shoot_ ;

    private Command xfercmd_ ;

    private Supplier<NoteDestination> destsupplier_ ;
    private Supplier<ShotType> shot_type_supplier_ ;

    private Trigger transfer_note_trigger_ ;
    private Trigger ready_for_shoot_trigger_ ;
    private boolean collect_after_manual_ ;
    // #endregion

    // #region constructor
    public IntakeShooterSubsystem(XeroRobot robot, DoubleSupplier distsupplier, Supplier<NoteDestination> destsupplier, Supplier<ShotType> shottype) throws Exception {
        super(robot, NAME) ;

        io_ = new IntakeShooterIOHardware(robot) ;
        inputs_ = new IntakeShooterIOInputsAutoLogged() ;

        destsupplier_ = destsupplier ;
        shot_type_supplier_ = shottype ;
        auto_mode_auto_shoot_ = false ;

        capture_timer_ = new XeroTimer("collect-timer", IntakeShooterConstants.kCollectDelayTime) ;
        reverse_timer_ = new XeroTimer("reverse-timer", IntakeShooterConstants.kReverseDelayTime) ;
        shoot_timer_ = new XeroTimer("shoot-timer", IntakeShooterConstants.Feeder.kShootTime) ;
        eject_forward_timer_ = new XeroTimer("eject-forward", IntakeShooterConstants.Shooter.kEjectForwardTime) ;
        eject_reverse_timer_ = new XeroTimer("eject-reverse", IntakeShooterConstants.Shooter.kEjectReverseTime) ;
        eject_pause_timer_ = new XeroTimer("eject-pause", IntakeShooterConstants.Shooter.kEjectPauseTime) ;

        io_.setTiltMotorPosition(io_.getTiltAbsoluteEncoderPosition());
        io_.setUpDownMotorPosition(IntakeShooterConstants.UpDown.Positions.kStowed);

        setTracking(false) ;
        distsupplier_ = distsupplier ;
        updown_pwl_ = new PieceWiseLinear(IntakeShooterConstants.UpDown.kPwlValues) ;
        tilt_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Tilt.kPwlValues) ;
        velocity_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Shooter.kPwlValues) ;

        state_ = State.Idle ;
        next_state_ = State.Invalid ;

        transfer_note_trigger_ = new Trigger(()-> canTransferNote()) ;
        ready_for_shoot_trigger_ = new Trigger(()-> state_ == State.HoldForShoot) ;

        need_stop_manipulator_ = false ;

        average_filter_ = LinearFilter.movingAverage(IntakeShooterConstants.Tilt.kMaxAbsoluteTiltMovingAverageTaps) ;
        last_time_ = Timer.getFPGATimestamp() ;
        last_value_ = inputs_.tiltAbsoluteEncoderPosition ;
    }
    // #endregion

    public void setTransferCmd(Command cmd) {
        xfercmd_ = cmd ;
    }

    public void setAutoModeAutoShoot(boolean b) {
        auto_mode_auto_shoot_ = b ;
    }

    // #region Shooter tuning related
    public String stateString() {
        return state_.toString() ;
    }

    public void startTuning() {
        if (state_ == State.Idle) {
            state_ = State.Tuning ;
        }
    }

    public void startFeeder() {
        io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kShootVoltage) ;
        state_ = State.WaitingForTunedNoteShot1 ;
    }

    public void stopFeeder() {
        io_.setFeederMotorVoltage(0.0) ;
        state_ = State.Tuning ;
    }    
    // #endregion

    // #region base subsystem code, mostly used for simulation
    public Map<String, TalonFX> getCTREMotors() {
        return io_.getCTREMotors() ;
    }

    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("has-note")) {
            v = new SettingsValue(has_note_) ;
        }

        return v ;
    }
    // #endregion

    // #region public triggers supplied by the subsystem

    public Trigger readyToShoot() {
        return ready_for_shoot_trigger_ ;
    }

    public Trigger readyForTransferNote() {
        return transfer_note_trigger_ ;
    }

    // #endregion

    // #region public methods to get the state of the subsystem
    public double getShooter1Velocity() {
        return inputs_.shooter1Velocity;
    }

    public double getShooter2Velocity() {
        return inputs_.shooter2Velocity;
    }

    public double getTilt() {
        return inputs_.tiltPosition ;
    }

    public double getUpDown() {
        return inputs_.updownPosition ;
    }

    public boolean hasNote() {
        return has_note_ ;
    }    

    public boolean isIdle() {
        return state_ == State.Idle ;
    }

    public boolean finishedShooterOnTransfer() {
        return state_ != State.TransferRunToShooterStop ;
    }

    public boolean isTuning() {
        return state_ == State.Tuning ;
    }

    public boolean isInTransferPosition() {
        return state_ == State.HoldingTransferPosition ;
    }    

    public boolean isTiltReady() {
        boolean b =  
            Math.abs(inputs_.tiltPosition - target_tilt_) < target_tilt_tol_ &&
            Math.abs(inputs_.tiltVelocity) < target_tilt_vel_ ;

        boolean absvel = true ;

        // if (Math.abs(average_value_) > IntakeShooterConstants.Tilt.kMaxAbsoluteTiltVelocity) {
        //     absvel = false ;
        // }
        // else {
        //     absvel = true ;
        // }
        return b & absvel ;
    }

    public boolean isUpDownReady() {
        return 
            Math.abs(inputs_.updownPosition - target_updown_) < target_updown_tol_ &&
            Math.abs(inputs_.updownVelocity) < target_updown_vel_ ;
    }

    public boolean isShooterReady() { 
        return 
            Math.abs(inputs_.shooter1Velocity - target_velocity_) < target_velocity_tol_ &&
            Math.abs(inputs_.shooter2Velocity - target_velocity_) < target_velocity_tol_ ;
    }    

    public boolean hasShotLeft() {
        return state_ != State.WaitForShotFinish && state_ != State.WaitingToShoot ;
    }    

    //
    // This is a HACK to allow the TrampSubsystem to know its time to stop 
    // the manipulator.  Stopping the manipulator is a function of the distance
    // traveled by the shooter wheels.
    //
    public boolean needStopManipulator() {
        return need_stop_manipulator_ ;
    }

    // #endregion

    // #region public methods to set the state of the subsystem
    public void setTiltToAngle(double t, double tiltpostol, double tiltveltol) {
        setTracking(false) ;
        setTiltTarget(t) ;

        target_tilt_tol_ = tiltpostol ;
        target_tilt_vel_ = tiltveltol ;
    }

    public void setUpDownToAngle(double pos, double updownpostol, double updownveltol) {
        setTracking(false) ;
        io_.setUpDownTargetPos(pos);

        target_updown_ = pos ;
        target_updown_tol_ = updownpostol ;
        target_updown_vel_ = updownveltol ;
    }
    //#endregion

    // #region public commands
    public Command collectCommand() {
        Command cmd = new FunctionalCommand(
                                ()->collect(),
                                () -> {},
                                (Boolean b) -> { if (b) stopCollect(); },
                                ()->isIdle() || hasNote()) ;
        cmd.setName("collect") ;        
        return cmd ;
    }

    public Command ejectCommand() {
        Command ret = new FunctionalCommand(
                                ()->eject(),
                                () -> {},
                                (Boolean b) -> {},
                                ()->isIdle()) ;
        ret.setName("eject") ;        
        return ret ;
    }

    public Command turtleCommand() {
        Command ret = new FunctionalCommand(
                                ()->turtle(),
                                () -> {},
                                (Boolean b) -> {},
                                ()->isIdle()) ;
        ret.setName("turtle") ;
        return ret ;
    }

    public Command manualShootCommand(double updown, double updownpostol, double updownveltol, double tilt, double tiltpostol, double tiltveltol, double shooter, double shooterveltol) {
        Command ret = new FunctionalCommand(
            ()->manualShoot(updown, updownpostol, updownveltol, tilt, tiltpostol, tiltveltol, shooter, shooterveltol, true, false),
            ()-> {},
            (Boolean b) -> { },
            () -> hasShotLeft());
        ret.setName("manual-shoot");
        return ret ;
    }

    public Command shootCommand() {
        return runOnce(this::finishShot) ;
    }

    // #endregion

    // #region public methods to perform actions

    //
    // If the intake is idle, and does not have a note, this method collects a new note.  This
    // is used from automodes to directly collect a note without any additional commands.
    //
    private boolean isStowing() {
        return (state_ == State.MoveTiltToPosition || state_ == State.MoveBothToPosition) && next_state_ == State.Idle ;
    }

    public void collect() {
        if (!has_note_ && (state_ == State.Idle || isStowing())) {
            //
            // Turn on the feeder
            //
            io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kCollectVoltage);

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
    // Set the shoot parameters for an upcoming manual shot.  This is used in automodes when we know as soon
    // as the collect is complete, we want to move to a shoot operation with the positions given below.
    //
    public void setManualShootParameters(double updown, double tilt) {
        auto_manual_shoot_tilt_ = tilt ;
        auto_manual_shoot_updown_ = updown ;
    }

    //
    // If the intake has a note, this method transfers the note to the manipulator.  This
    // method assumes the intake/shooter and elevator/manipulator are in the correct positions
    // before being called.  If they are in the transfer position, the state will be HoldingTransferPosition.
    //
    public void doTransferNote() {
        if (has_note_ && state_ == State.HoldingTransferPosition) {
            io_.setFeederMotorVoltage(3);
            setShooterVelocity(IntakeShooterConstants.Shooter.kTransferVelocity, 10.0);
            need_stop_manipulator_ = false ;

            if (isNoteDetected()) {
                //
                // The note is already sitting on the sensor.  We just wait for the note to
                // move off the sensor
                //
                state_ = State.TransferWaitForNoNoteCenter ;
            }
            else {
                //
                // The note is not on the sensor, so the sensor is assumed to be sitting in the middle of the note.
                // We wait until we sense the note and then look for the note to move off the sensor.
                //
                state_ = State.TransferWaitForNote ;
            }
        }
    }

    //
    // This method moves the intake/shooter and the elevator/arm to the transfer positions.
    //
    public void moveToTransferPosition() {
        if (has_note_) {
            //
            // Move the updown and tilt to the transfer position
            //
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kTransfer, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kTransfer, 10.0, 10.0) ;
            
            //
            // And set the state for after we reach the transfer position
            //
            next_state_ = State.HoldingTransferPosition ;                         
        }
    }

    public void manualShoot(double updown, double updownpostol, double updownveltol, double tilt, double tiltpostol, double tiltveltol, double shooter, double shooterveltol, boolean immd, boolean collect) {
        setTracking(false);
        gotoPosition(updown, updownpostol, updownveltol, tilt, tiltpostol, tiltveltol) ;
        setShooterVelocity(shooter, shooterveltol);

        //
        // If true, after the shot is complete, we will move to the collect position.  Otherswise, we will move to the stowed position.
        //
        collect_after_manual_ = collect ;

        //
        // If manual shoot was called and we want to shoot as soon as possible, then we set immd to true.  This is most
        // useful in automodes to shoot now without any user interaction.  If immd is false, every things gets ready for the
        // shot, but we wait on the drive team to press the button to shoot.
        //
        if (immd) {
            next_state_ = State.WaitingToShoot ;
        }
        else {
            next_state_ = State.HoldForShoot ;
        }
    }

    public void finishShot() {
        //
        // This locks in the parameteters for the shot.
        //
        setTracking(false);
        state_ = State.WaitingToShoot ;
    }

    public void eject() {
        if (xfercmd_ != null) {
            xfercmd_.cancel() ;
        }
        setTracking(false);
        gotoPosition(IntakeShooterConstants.UpDown.Positions.kEject, 5.0, 100.0, 
                     IntakeShooterConstants.Tilt.Positions.kEject, 8.0, 100.0) ;
        state_ = State.GoToEjectPosition ;
    }

    //
    // Used by auto modes to tell the intake subsystem it was preloaded with a note.
    //
    public void setHasNote(boolean b) {
        has_note_ = b ;
    }    

    public void setShooterVelocity(double vel, double veltol) {

        if (Math.abs(vel) < 0.1) {
            //
            // Setting shooter velocity to zero causes the shooter wheels to 
            // vibrate.  When we see this really low velocity, we just turn off
            // the motors by setting the voltage to zero.
            io_.setShooter1MotorVoltage(0.0) ;
            io_.setShooter2MotorVoltage(0.0) ;
        }
        else {
            io_.setShooter1Velocity(vel);
            io_.setShooter2Velocity(vel);
        }

        target_velocity_tol_ = veltol ;
        target_velocity_ = vel ;
    }
    // #endregion

    // #region private implementation methods

    private void setTracking(boolean b) {
        tracking_ = b ;
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

    public void gotoPositionThenIdle(double updown, double updowntol, double updownvel, double tilt, double tilttol, double tiltvel) {
        next_state_ = State.Idle ;
        gotoPosition(updown, updowntol, updownvel, tilt, tilttol, tiltvel);
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


    //
    // If we are in the idle state, this method moves the mechanisms to the stowed position.
    //
    private void turtle() {
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
    // If we are collecting, this stops the collect operation.
    //
    private void stopCollect() {
        if (state_ == State.MoveTiltToPosition || state_ == State.MoveBothToPosition || state_ == State.WaitForNote) {
            //
            // Turn off the feeder and stop the tilt and updown
            //
            io_.setFeederMotorVoltage(0.0) ;
            io_.setTiltTargetPos(false, inputs_.tiltPosition);
            io_.setUpDownTargetPos(inputs_.updownPosition);
            reverse_timer_.start() ;

            state_ = State.WaitForReverse ;
        }
    }    

    private boolean isNoteDetected() {
        return inputs_.noteSensor ^ IntakeShooterConstants.NoteSensor.kInverted ;
    }

    private boolean canTransferNote() {
        boolean ret = false ;

        if (!DriverStation.isAutonomous()) {
            NoteDestination dest = getNoteDestination() ;
            ret = has_note_ && 
                     (dest == NoteDestination.Trap ||  dest == NoteDestination.Amp) &&
                     (state_ == State.Idle || state_ == State.MoveTiltToPosition || state_ == State.MoveBothToPosition || state_ == State.HoldForShoot) ;
        }
        return ret;
    }

    private NoteDestination getNoteDestination() {
        NoteDestination ret = NoteDestination.Speaker ;
        if (DriverStation.isAutonomous()) {
            if (auto_mode_auto_shoot_)
                ret = NoteDestination.Speaker ;
            else 
                ret = NoteDestination.AutoDefinedSpeaker ;
        }
        else if (destsupplier_ != null) {
            ret = destsupplier_.get() ;
        }

        return ret ;
    }

    private ShotType getShotType() {
        if (shot_type_supplier_ != null)
            return shot_type_supplier_.get() ;

        return ShotType.Auto ;
    }

    private void setUpDownTarget(double pos) {
        target_updown_ = pos ;
        io_.setUpDownTargetPos(pos);
    }

    private void setTiltTarget(double pos) {
        io_.setTiltTargetPos(tracking_, pos);
        target_tilt_ = pos ;
    }

    private void trackTargetDistance() {
        double dist = distsupplier_.getAsDouble() ;

        double updown = updown_pwl_.getValue(dist) ;
        double tilt = tilt_pwl_.getValue(dist) ;
        double velocity = velocity_pwl_.getValue(dist) ;

        setUpDownTarget(updown);
        setTiltTarget(tilt) ;
        setShooterVelocity(velocity, IntakeShooterConstants.Shooter.kAutoShootVelocityTol) ;
    }    

    // #endregion

    // #region methods that implement states in the state machine


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
        if (next_state_ == State.WaitForNote && inputs_.fallingEdge) {
            has_note_ = true ;
            capture_timer_.start() ;
            state_ = State.WaitForCapture ;
        }
        else if (isTiltReady() && isUpDownReady()) {
            state_ = next_state_ ;
            next_state_ = State.Invalid ;
        }
    }    

    private void waitForNoteState() {
        if (inputs_.fallingEdge) {
            //
            // Set the note flag, start moving the intake to the target position, and wait for the feeder to run
            // until we have the note fully captured.
            //
            has_note_ = true ;
            capture_timer_.start() ;
            state_ = State.WaitForCapture ;
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

    private void waitForCaptureState() {
        if (capture_timer_.isExpired()) {
            //
            // Turn off the feeder
            //
            io_.setFeederMotorVoltage(0.0);

            //
            // Move the updown and tilt to the stowed position
            //
            double updown ;
            double tilt ;
            
            NoteDestination dest = getNoteDestination();
            switch(dest) {
                case AutoDefinedSpeaker:
                    next_state_ = State.Idle ;
                    gotoPosition(auto_manual_shoot_updown_, Double.NaN, Double.NaN, auto_manual_shoot_tilt_, Double.NaN, Double.NaN) ; 
                    break ;
                case Speaker:
                    {
                        ShotType type = getShotType() ;
                        if (type == ShotType.Auto) {
                            updown = IntakeShooterConstants.UpDown.Positions.kStartTracking ;
                            tilt = IntakeShooterConstants.Tilt.Positions.kStartTracking;          
                            gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;
                            setTracking(true) ;
                            next_state_ = State.HoldForShoot ;
                        }
                        else if (type == ShotType.Podium) {
                            manualShoot(IntakeShooterConstants.ManualShotPodium.kUpDownPos, 
                                        IntakeShooterConstants.ManualShotPodium.kUpDownPosTolerance,
                                        IntakeShooterConstants.ManualShotPodium.kUpDownVelTolerance,
                                        IntakeShooterConstants.ManualShotPodium.kTiltPos,
                                        IntakeShooterConstants.ManualShotPodium.kTiltPosTolerance,
                                        IntakeShooterConstants.ManualShotPodium.kTiltVelTolerance,
                                        IntakeShooterConstants.ManualShotPodium.kShooterVel,
                                        IntakeShooterConstants.ManualShotPodium.kShooterVelTolerance,
                                        false, false) ;
                        }
                        else if (type == ShotType.Subwoofer) {
                            manualShoot(IntakeShooterConstants.ManualShotSubwoofer.kUpDownPos, 
                                        IntakeShooterConstants.ManualShotSubwoofer.kUpDownPosTolerance,
                                        IntakeShooterConstants.ManualShotSubwoofer.kUpDownVelTolerance,
                                        IntakeShooterConstants.ManualShotSubwoofer.kTiltPos,
                                        IntakeShooterConstants.ManualShotSubwoofer.kTiltPosTolerance,
                                        IntakeShooterConstants.ManualShotSubwoofer.kTiltVelTolerance,
                                        IntakeShooterConstants.ManualShotSubwoofer.kShooterVel,
                                        IntakeShooterConstants.ManualShotSubwoofer.kShooterVelTolerance,
                                        false, false) ;
                        }
                    }
                    break ;

                case Trap:
                case Amp:
                    updown = IntakeShooterConstants.UpDown.Positions.kTransfer ;
                    tilt = IntakeShooterConstants.Tilt.Positions.kTransfer ;
                    next_state_ = State.Idle ;                        
                    gotoPosition(updown, 3.0, 10.0, tilt, 3.0, 1e32) ;                    
                    break ;

                default:
                    updown = IntakeShooterConstants.UpDown.Positions.kStowed ;
                    tilt = IntakeShooterConstants.Tilt.Positions.kStowed ;
                    next_state_ = State.Idle ;
                    gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;                    
                    break ;
            }

            //
            // Start the shooter wheels so that the shooter is up to speed when the updown/tilt reach the
            // transfer position
            //
            if (dest == NoteDestination.Amp) {
                setShooterVelocity(IntakeShooterConstants.Shooter.kTransferVelocity, IntakeShooterConstants.Shooter.kTransferVelocityTol) ;
            }
        }
    }


    public void waitingToShootState() {
        if (isTiltReady() && isUpDownReady() && isShooterReady()) {
            io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kShootVoltage) ;
            shoot_timer_.start() ;
            state_ = State.WaitForShotFinish ;
        }
    }

    private void waitForShotFinishState() {
        if (shoot_timer_.isExpired()) {
            //
            // Indicate the note has left the robot
            //
            has_note_ = false ;

            //
            // Turn off the feeder and shooter
            //
            io_.setFeederMotorVoltage(0.0);
            io_.setShooter1MotorVoltage(0.0);
            io_.setShooter2MotorVoltage(0.0);

            setTracking(false) ;

            if (!collect_after_manual_) {
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
            else {
                //
                // Force the intake to the idle state, so impose a collect operation
                // as soon after shot is complete as possible
                //
                state_ = State.Idle ;
                collect() ;
                collect_after_manual_ = false ;
            }
        }
    }

    private void ejectForwardState() {
        if (eject_forward_timer_.isExpired()) {
            setShooterVelocity(0.0, 0.0);
            io_.setFeederMotorVoltage(0.0) ;
            eject_pause_timer_.start() ;
            state_ = State.EjectPause ;
        }
    }

    private void ejectPauseState() {
        if (eject_pause_timer_.isExpired()) {
            setShooterVelocity(-IntakeShooterConstants.Shooter.kEjectVelocity, 10.0) ;
            io_.setFeederMotorVoltage(-IntakeShooterConstants.Feeder.kEjectVoltage) ;
            eject_reverse_timer_.start() ;
            state_ = State.EjectReverse ;
        }
    }

    private void ejectReverseState() {
        if (eject_reverse_timer_.isExpired()) {
            setShooterVelocity(0.0, 0.0);
            io_.setFeederMotorVoltage(0.0);
            next_state_ = State.Idle ;
            gotoPosition(IntakeShooterConstants.UpDown.Positions.kStowed, Double.NaN, Double.NaN,
                         IntakeShooterConstants.Tilt.Positions.kStowed, Double.NaN, Double.NaN) ;
        }
    }
    // #endregion

    int counter = 0 ;
    double lasttime = 0.0 ;
    boolean lastone = false ;

    // #region periodic method that evaluates the state machine each robot loop
    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("intake-shooter", inputs_);

        if (average_filter_ != null) {
            double now = Timer.getFPGATimestamp() ;
            double vel = (inputs_.tiltAbsoluteEncoderPosition - last_value_) / (now - last_time_) ;
            average_value_ = average_filter_.calculate(vel) ;

            Logger.recordOutput("tilt-abs-velocity", average_value_) ;
            Logger.recordOutput("tilt-raw-vel", vel) ;

            last_time_ = now ;
            last_value_ = inputs_.tiltAbsoluteEncoderPosition ;
        }

        switch(state_) {
            case Idle:
            case Invalid:
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
                // We leave this state when the TransferCommand asks us to directly.
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

            case TransferWaitForNote:
                if (inputs_.risingEdge && inputs_.fallingEdge) {
                    //
                    // The note completely passed the sensor since the last robot loop
                    //
                    state_ = State.TransferRunToManipulatorStop ;
                    transfer_start_pos_ = io_.getShooterPositionAtRisingEdge() ;                    
                }
                else if (inputs_.fallingEdge) {
                    //
                    // The leading edge of the note passed the sensor, wait for the trailing edge
                    //
                    state_ = State.TransferWaitForNoNote2 ;
                }
                break ;

            case TransferWaitForNoNoteCenter:
                if (inputs_.risingEdge){
                    state_ = State.TransferWaitForNote ;
                    
                }
                break ;

            case TransferWaitForNoNote2:
                if (inputs_.risingEdge) {
                    state_ = State.TransferRunToManipulatorStop ;
                    transfer_start_pos_ = io_.getShooterPositionAtRisingEdge() ;
                }                
                break ;

            case TransferRunToManipulatorStop:
                if (inputs_.shooter1Position - transfer_start_pos_ > IntakeShooterConstants.Shooter.kTransferLength) {
                    need_stop_manipulator_ = true ;
                    has_note_ = false ;                    
                    state_ = State.TransferRunToShooterStop ;
                }
                break ;

            case TransferRunToShooterStop:
                if (inputs_.shooter1Position - transfer_start_pos_ - IntakeShooterConstants.Shooter.kTransferLength > IntakeShooterConstants.Shooter.kTransferContLength) {
                    io_.setFeederMotorVoltage(0.0);
                    setShooterVelocity(0.0, 0.0);
                    next_state_ = State.Idle ;                    
                    gotoPosition(IntakeShooterConstants.UpDown.Positions.kStowed, Double.NaN, Double.NaN,
                                 IntakeShooterConstants.Tilt.Positions.kStowed, Double.NaN, Double.NaN) ;
                }
                break ;
                
            case GoToEjectPosition:
                if (isTiltReady() && isUpDownReady()) {
                    setShooterVelocity(IntakeShooterConstants.Shooter.kEjectVelocity, 10.0) ;
                    io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kEjectVoltage) ;
                    eject_forward_timer_.start() ;
                    has_note_ = false ;
                    state_ = State.EjectForward ;
                }
                break ;
            
            case HoldForShoot:
                break ;

            case Tuning:
                break ;

            case WaitingForTunedNoteShot1:
                if (inputs_.risingEdge || inputs_.fallingEdge) {
                    shoot_timer_.start() ;
                    state_ = State.WaitingForTunedNoteShot2 ;
                }
                break ;

            case WaitingForTunedNoteShot2:
                if (shoot_timer_.isExpired()) {
                    state_ = State.Tuning ;
                }
                break ;
        }

        if (tracking_ && distsupplier_ != null) {
            trackTargetDistance() ;
        }

        String ststr = state_.toString() ;

        if (state_ == State.MoveTiltToPosition) {
            ststr += ":" + target_tilt_ ;
        }
        else if (state_ == State.MoveBothToPosition) {
            ststr += ":" + target_updown_ + ":" + target_tilt_ ;
        }

        AllegroContainer container = (AllegroContainer)getRobot().getContainer() ;
        OISubsystem oi = container.getOI() ;

        if (oi != null) {
            oi.setLEDState(OILed.ShooterReady, isShooterReady()) ;
            oi.setLEDState(OILed.TiltReady, isTiltReady()) ;
        }

        AllegroContainer.componentVisualizer.setUpdownAngle(Units.Degrees.of(-inputs_.updownPosition));
        AllegroContainer.componentVisualizer.setTiltAngle(Units.Degrees.of(-inputs_.tiltAbsoluteEncoderPosition + 90));

        if (getVerbose()) {
            Logger.recordOutput("intake:state", ststr);
            Logger.recordOutput("intake:next-state", next_state_) ;

            if (state_ == State.MoveBothToPosition || tracking_) {
                Logger.recordOutput("intake:updown-target", target_updown_) ;
            }

            if (state_ == State.MoveBothToPosition || state_ == State.MoveTiltToPosition || tracking_) {
                Logger.recordOutput("intake:tilt-target", target_tilt_) ;
            }

            Logger.recordOutput("intake:is-tilt-ready", isTiltReady());
            Logger.recordOutput("intake:is-updown-ready", isUpDownReady());
            Logger.recordOutput("intake:is-shooter-ready", isShooterReady());
            Logger.recordOutput("intake:shooter_target", target_velocity_) ;
            Logger.recordOutput("intake:readyForShoot", isShooterReady() && isTiltReady() && isUpDownReady()) ;

            Logger.recordOutput("intake:has-note", has_note_);
            Logger.recordOutput("intake:tracking", tracking_);
            Logger.recordOutput("intake:note-dest", getNoteDestination()) ;
            Logger.recordOutput("intake:shot-type", getShotType()) ;
            Logger.recordOutput("intake:feederVoltage", io_.getFeederMotorVoltage()) ;
            Logger.recordOutput("intake:tracking", tracking_) ;
            Logger.recordOutput("intake:needStopManip", need_stop_manipulator_);

            Logger.recordOutput("intake:cappos", io_.getShooterPositionAtRisingEdge());
        }        
    }

    // #endregion

    // #region methods assocaited with characterization of the subsytsems

    private SysIdRoutine tiltSysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(7) ;
        Measure<Time> to = Units.Seconds.of(10.0) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setTiltMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logTiltMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }

    private SysIdRoutine upDownSysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(2) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setUpDownMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logUpdownMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }

    private SysIdRoutine shooter1SysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(3) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setShooter1MotorVoltage(volts.magnitude()),
                                        (log) -> io_.logShooter1Motor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }
    
    private SysIdRoutine shooter2SysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(3) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setShooter2MotorVoltage(volts.magnitude()),
                                        (log) -> io_.logShooter2Motor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }

    public Command upDownSysIdQuasistatic(SysIdRoutine.Direction dir) {
        Command tunecmd = upDownSysIdRoutine().quasistatic(dir) ;
        return new SequentialCommandGroup(
            new IntakeTiltGotoAngleCommand(this, -20, 5.0, 5.0),
            tunecmd) ;
    }

    public Command upDownSysIdDynamic(SysIdRoutine.Direction dir) {
        Command tunecmd = upDownSysIdRoutine().dynamic(dir) ;
        return new SequentialCommandGroup(
            new IntakeTiltGotoAngleCommand(this, -20, 5.0, 5.0),
            tunecmd) ;
    }

    public Command tiltSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return tiltSysIdRoutine().quasistatic(dir) ;
    }
    
    public Command tiltSysIdDynamic(SysIdRoutine.Direction dir) {
        return tiltSysIdRoutine().dynamic(dir) ;
    }

    public Command shooter1SysIdQuasistatic(SysIdRoutine.Direction dir) {
        return shooter1SysIdRoutine().quasistatic(dir) ;
    }

    public Command shooter1SysIdDynamic(SysIdRoutine.Direction dir) {
        return shooter1SysIdRoutine().dynamic(dir) ;
    }

    public Command shooter2SysIdQuasistatic(SysIdRoutine.Direction dir) {
        return shooter2SysIdRoutine().quasistatic(dir) ;
    }

    public Command shooter2SysIdDynamic(SysIdRoutine.Direction dir) {
        return shooter2SysIdRoutine().dynamic(dir) ;
    }

    // #endregion
}

