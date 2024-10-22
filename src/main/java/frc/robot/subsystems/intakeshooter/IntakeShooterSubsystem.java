package frc.robot.subsystems.intakeshooter ;

import static edu.wpi.first.units.Units.Degrees;

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
import frc.robot.subsystems.oi.OISubsystem.LEDState;
import frc.robot.subsystems.oi.OISubsystem.OILed;
import frc.robot.util.ComponentVisualizer;
import frc.robot.util.NoteVisualizer;

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

        StartEjectOperations,
        EjectForward,
        EjectPause,
        EjectReverse,

        HoldForShoot,

        Tuning,

        TransferringNote,
        TransferFinish1,
        TransferFinish2,

        WaitingForTunedNoteShot1,
        WaitingForTunedNoteShot2,        
    }
    // #endregion

    // #region private member variables
    private IntakeShooterIO io_ ;
    private IntakeShooterIOInputsAutoLogged inputs_ ;

    private ComponentVisualizer visualizer_;
    private NoteVisualizer noteVisualizer_;

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
    private boolean ejecting_ ;
    private DoubleSupplier distsupplier_ ;
    private PieceWiseLinear updown_pwl_ ;
    private PieceWiseLinear tilt_pwl_ ;
    private PieceWiseLinear velocity_pwl_ ;
    private LinearFilter average_filter_ ;
    private double average_value_ ;
    private double last_value_ ;
    private double last_time_ ;

    private boolean has_note_ ;

    private XeroTimer capture_timer_ ;
    private XeroTimer reverse_timer_ ;
    private XeroTimer shoot_timer_ ;
    private XeroTimer eject_forward_timer_ ;
    private XeroTimer eject_reverse_timer_ ;
    private XeroTimer eject_pause_timer_ ;

    private State state_ ;
    private State next_state_ ;
    private boolean auto_mode_auto_shoot_ ;


    private Supplier<NoteDestination> destsupplier_ ;
    private Supplier<ShotType> shot_type_supplier_ ;

    private boolean encoders_synced_ ;

    private Trigger transfer_note_trigger_ ;
    private Trigger ready_for_shoot_trigger_ ;
    private boolean collect_after_manual_ ;

    private boolean firing_ ;
    // #endregion

    // #region constructor
    public IntakeShooterSubsystem(XeroRobot robot, DoubleSupplier distsupplier, Supplier<NoteDestination> destsupplier, Supplier<ShotType> shottype, ComponentVisualizer visualizer, NoteVisualizer noteVisualizer) throws Exception {
        super(robot, NAME) ;

        io_ = new IntakeShooterIOHardware(robot) ;
        inputs_ = new IntakeShooterIOInputsAutoLogged() ;

        destsupplier_ = destsupplier ;
        shot_type_supplier_ = shottype ;
        auto_mode_auto_shoot_ = false ;

        firing_ = false ;

        capture_timer_ = new XeroTimer("collect-timer", IntakeShooterConstants.kCollectDelayTime) ;
        reverse_timer_ = new XeroTimer("reverse-timer", IntakeShooterConstants.kReverseDelayTime) ;
        shoot_timer_ = new XeroTimer("shoot-timer", IntakeShooterConstants.Feeder.kShootTime) ;
        eject_forward_timer_ = new XeroTimer("eject-forward", IntakeShooterConstants.Shooter.kEjectForwardTime) ;
        eject_reverse_timer_ = new XeroTimer("eject-reverse", IntakeShooterConstants.Shooter.kEjectReverseTime) ;
        eject_pause_timer_ = new XeroTimer("eject-pause", IntakeShooterConstants.Shooter.kEjectPauseTime) ;


        io_.setUpDownMotorPosition(IntakeShooterConstants.UpDown.Positions.kStowed);

        setTracking(false) ;
        distsupplier_ = distsupplier ;
        updown_pwl_ = new PieceWiseLinear(IntakeShooterConstants.UpDown.kPwlValues) ;
        tilt_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Tilt.kPwlValues) ;
        velocity_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Shooter.kPwlValues) ;

        state_ = State.Idle ;
        next_state_ = State.Invalid ;

        transfer_note_trigger_ = new Trigger(()-> canTransferNote()) ;
        ready_for_shoot_trigger_ = new Trigger(()-> state_ == State.HoldForShoot || tracking_ ) ;

        average_filter_ = LinearFilter.movingAverage(IntakeShooterConstants.Tilt.kMaxAbsoluteTiltMovingAverageTaps) ;
        last_time_ = Timer.getFPGATimestamp() ;
        last_value_ = inputs_.tiltAbsoluteEncoderPosition ;

        encoders_synced_ = false ;

        visualizer_ = visualizer;
        noteVisualizer_ = noteVisualizer;
    }
    // #endregion

    public void syncTiltEncoders(boolean b) {
        if (b) {
            io_.setTiltMotorPosition(inputs_.tiltAbsoluteEncoderPositionMedian) ;
        }
        else {
            io_.setTiltMotorPosition(inputs_.tiltAbsoluteEncoderPosition) ;
        }
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

    public boolean isFinishTransfer2() {
        return state_ == State.TransferFinish2 ;
    }

    public boolean isTuning() {
        return state_ == State.Tuning ;
    }

    public boolean isInTransferPosition() {
        return state_ == State.HoldingTransferPosition ;
    }    

    public void stow() {
        gotoPosition(State.Idle,
                        IntakeShooterConstants.UpDown.Positions.kStowed, 
                        IntakeShooterConstants.Tilt.Positions.kStowed) ;        
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

    // #endregion

    // #region public methods to set the state of the subsystem
    public void setTiltToAngle(double t, double tiltpostol, double tiltveltol) {
        setTracking(false) ;
        setTiltTarget(t, tiltpostol, tiltveltol) ;
    }

    public void setUpDownToAngle(double pos, double updownpostol, double updownveltol) {
        setTracking(false) ;
        setUpDownTarget(pos, updownpostol, updownveltol);
    }
    //#endregion

    // #region public commands
    public Command collectCommand() {
        Command cmd = new FunctionalCommand(
                                ()->collect(),
                                () -> {},
                                (Boolean b) -> { if (b) stopCollect(); },
                                ()->isIdle() || hasNote(),
                                this) ;
        cmd.setName("collect") ;        
        return cmd ;
    }

    public Command ejectCommand() {
        Command ret = new FunctionalCommand(
                                ()->eject(),
                                () -> {},
                                (Boolean b) -> {},
                                ()-> { return isIdle() || isStowing(); } ,
                                this) ;
        ret.setName("eject") ;        
        return ret ;
    }

    public Command turtleCommand() {
        Command ret = new FunctionalCommand(
                                ()->turtle(false),
                                () -> {},
                                (Boolean b) -> {},
                                ()->isIdle() ,
                                this) ;
        ret.setName("turtle") ;
        return ret ;
    }

    public Command manualShootCommand(double updown, double updownpostol, double updownveltol, double tilt, double tiltpostol, double tiltveltol, double shooter, double shooterveltol) {
        Command ret = new FunctionalCommand(
            ()->manualShoot(updown, updownpostol, updownveltol, tilt, tiltpostol, tiltveltol, shooter, shooterveltol, true, false),
            ()-> {},
            (Boolean b) -> { },
            () -> hasShotLeft(), 
            this);
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
            gotoPosition(State.WaitForNote,
                         IntakeShooterConstants.UpDown.Positions.kCollect, 
                         IntakeShooterConstants.Tilt.Positions.kCollect) ;
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
            setShooterVelocity(IntakeShooterConstants.Shooter.kTransferVelocity, IntakeShooterConstants.Shooter.kTransferVelocityTol);
            state_ = State.TransferringNote ;
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
            gotoPosition(State.HoldingTransferPosition, 
                         IntakeShooterConstants.UpDown.Positions.kTransfer, 
                         IntakeShooterConstants.UpDown.Positions.kTransferPosTol,
                         IntakeShooterConstants.UpDown.Positions.kTransferPosTol,
                         IntakeShooterConstants.Tilt.Positions.kTransfer, 
                         IntakeShooterConstants.Tilt.Positions.kTransferPosTol,
                         IntakeShooterConstants.Tilt.Positions.kTransferVelTol) ;
        }
    }

    public void manualShoot(double updown, double updownpostol, double updownveltol, double tilt, double tiltpostol, double tiltveltol, double shooter, double shooterveltol, boolean immd, boolean collect) {
        State st = immd ? State.WaitingToShoot : State.HoldForShoot ; 
        setTracking(false);
        gotoPosition(st, updown, updownpostol, updownveltol, tilt, tiltpostol, tiltveltol) ;
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

    }

    public void finishShot() {
        //
        // This locks in the parameteters for the shot.
        //
        setTracking(false);
        state_ = State.WaitingToShoot ;
    }

    public void eject() {
        ejecting_ = true ;
        setTracking(false);
        gotoPosition(State.StartEjectOperations,
                     IntakeShooterConstants.UpDown.Positions.kEject, 
                     IntakeShooterConstants.Tilt.Positions.kEject) ;
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

    private void gotoPosition(State st, double updown, double tilt) {
        gotoPosition(st, updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;
    }

    private void gotoPosition(State st, double updown, double updowntol, double updownvel, double tilt, double tilttol, double tiltvel) {
        next_state_ = st ;

        //
        // Compute the tilt position that would be required based on where the current updown
        // is located to be on the "standard" path.  This is used to determine if we need to move the
        // tilt alone first, or can move the updown and tilt together.
        //
        double desired_tilt = computeTiltFromUpdown(inputs_.updownPosition) ;
        if (Math.abs(inputs_.tiltPosition - desired_tilt) > IntakeShooterConstants.Tilt.kAllowedDeviationFromTrack) {
            //
            // First align tilt to the desired position, then move them to the desired position together
            // We set the up down to its current position so it doesn't move, but this also sets the tolerance
            // values to be used when we switch to moving the updown
            //
            setUpDownTarget(inputs_.updownPosition, updowntol, updownvel);
            setTiltTarget(desired_tilt, tilttol, tiltvel) ;
            state_ = State.MoveTiltToPosition ;            
        }
        else {
            //
            // Single move of updown/tilt together
            //
            setUpDownTarget(updown, updowntol, updownvel) ;
            setTiltTarget(tilt, tilttol, tiltvel) ;
            state_ = State.MoveBothToPosition ;            
        }
    }


    //
    // If we are in the idle state, this method moves the mechanisms to the stowed position.
    //
    public void turtle(boolean force) {
        if (state_ == State.Idle || force) {
            //
            // Move the updown and tilt to the collect position
            //
            gotoPosition(State.Idle,
                         IntakeShooterConstants.UpDown.Positions.kStowed, 
                         IntakeShooterConstants.Tilt.Positions.kStowed) ;
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
            io_.setTiltTargetPos(inputs_.tiltPosition);
            io_.setUpDownTargetPos(inputs_.updownPosition);
            reverse_timer_.start() ;

            state_ = State.WaitForReverse ;
        }
    }    

    private boolean canTransferNote() {
        boolean ret = false ;

        if (!DriverStation.isAutonomous()) {
            NoteDestination dest = getNoteDestination() ;
            ret = has_note_ && !tracking_ && !ejecting_ &&
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

    private void setUpDownTarget(double pos, double postol, double veltol) {
        target_updown_ = pos ;

        if (Double.isNaN(postol))
            target_updown_tol_ = IntakeShooterConstants.UpDown.kTargetPosTolerance ;
        else
            target_updown_tol_ = postol ;

        if (Double.isNaN(veltol))
            target_updown_vel_ = IntakeShooterConstants.UpDown.kTargetVelTolerance ;
        else
            target_updown_vel_ = veltol ;

        io_.setUpDownTargetPos(pos);
    }

    private void setTiltTarget(double pos, double postol, double veltol) {
        target_tilt_ = pos ;

        if (Double.isNaN(postol))
            target_tilt_tol_ = IntakeShooterConstants.Tilt.kTargetPosTolerance ;
        else
            target_tilt_tol_ = postol ;

        if (Double.isNaN(veltol))
            target_tilt_vel_ = IntakeShooterConstants.Tilt.kTargetVelTolerance ;
        else
            target_tilt_vel_ = veltol ;

        io_.setTiltTargetPos(pos);        
    }

    private void trackTargetDistance() {
        double dist = distsupplier_.getAsDouble() ;

        double updown = updown_pwl_.getValue(dist) ;
        double tilt = tilt_pwl_.getValue(dist) ;
        double velocity = velocity_pwl_.getValue(dist) ;

        if (dist > 5.0) {
            velocity = 0.0 ;
        }

        setUpDownTarget(updown, IntakeShooterConstants.UpDown.kTargetPosTolerance, IntakeShooterConstants.UpDown.kTargetVelTolerance) ;
        setTiltTarget(tilt, IntakeShooterConstants.Tilt.kTargetPosTolerance, IntakeShooterConstants.Tilt.kTargetVelTolerance) ;
        setShooterVelocity(velocity, IntakeShooterConstants.Shooter.kAutoShootVelocityTol) ;
    }    

    // #endregion

    // #region methods that implement states in the state machine


    //
    // These methods corespond to the the states the subsystem can be in
    //
    private void moveTiltToPositionState() {
        if (isTiltReady()) {
            //
            // This is part of a goto position call the tolerances are already set.  We just
            // reuse the existing values.
            //
            setUpDownTarget(next_updown_, target_updown_tol_, target_updown_vel_) ;
            setTiltTarget(next_tilt_, target_tilt_tol_, target_tilt_vel_) ;
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
            next_state_ = State.Invalid ;
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
            gotoPosition(State.Idle,
                         IntakeShooterConstants.UpDown.Positions.kStowed, 
                         IntakeShooterConstants.Tilt.Positions.kStowed) ;
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
            NoteDestination dest = getNoteDestination();
            switch(dest) {
                case AutoDefinedSpeaker:
                    gotoPosition(State.Idle, auto_manual_shoot_updown_, auto_manual_shoot_tilt_) ; 
                    break ;
                case Speaker:
                    {
                        ShotType type = getShotType() ;
                        if (type == ShotType.Auto) {
                            gotoPosition(State.HoldForShoot,
                                         IntakeShooterConstants.UpDown.Positions.kStartTracking,
                                         IntakeShooterConstants.Tilt.Positions.kStartTracking) ;
                            setTracking(true) ;
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
                    gotoPosition(State.Idle,
                                 IntakeShooterConstants.UpDown.Positions.kTransfer,
                                 IntakeShooterConstants.UpDown.Positions.kTransferPosTol, 
                                 IntakeShooterConstants.UpDown.Positions.kTransferVelTol,
                                 IntakeShooterConstants.Tilt.Positions.kTransfer,
                                 IntakeShooterConstants.Tilt.Positions.kTransferPosTol,
                                 IntakeShooterConstants.Tilt.Positions.kTransferVelTol) ;  
                    break ;

                default:
                    gotoPosition(State.Idle,
                                 IntakeShooterConstants.UpDown.Positions.kStowed, 
                                 IntakeShooterConstants.Tilt.Positions.kStowed) ;
                    break ;
            }

            //
            // Start the shooter wheels so that the shooter is up to speed when the updown/tilt reach the
            // transfer position
            //
            if (dest == NoteDestination.Amp || dest == NoteDestination.Trap) {
                setShooterVelocity(IntakeShooterConstants.Shooter.kTransferVelocity, IntakeShooterConstants.Shooter.kTransferVelocityTol) ;
            }
        }
    }

    public void waitingToShootState() {
        if (isTiltReady() && isUpDownReady() && isShooterReady()) {
            firing_ = true ;
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

            firing_ = false ;

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
                gotoPosition(State.Idle, 
                             IntakeShooterConstants.UpDown.Positions.kStowed, 
                             IntakeShooterConstants.Tilt.Positions.kStowed) ;
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
            setShooterVelocity(IntakeShooterConstants.Shooter.kEjectVelocity, 10.0) ;
            io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kEjectVoltage) ;
            eject_reverse_timer_.start() ;
            state_ = State.EjectReverse ;
        }
    }

    private void ejectReverseState() {
        if (eject_reverse_timer_.isExpired()) {
            setShooterVelocity(0.0, 0.0);
            io_.setFeederMotorVoltage(0.0);
            gotoPosition(State.Idle, 
                         IntakeShooterConstants.UpDown.Positions.kStowed, 
                         IntakeShooterConstants.Tilt.Positions.kStowed) ;
            ejecting_ = false ;
        }
    }
    // #endregion

    public void endNoteTransfer() {
        transfer_start_pos_ = inputs_.shooter1Position ;
        has_note_ = false ;
        state_ = State.TransferFinish1 ;
    }

    // #region periodic method that evaluates the state machine each robot loop
    @Override
    public void periodic() {
        super.startPeriodic() ;
        io_.updateInputs(inputs_);

        boolean synced = false ;
        double motortilt = inputs_.tiltPosition ;
        if (!encoders_synced_ && getRobot().isEnabled()) {
            syncTiltEncoders(true) ;
            motortilt = inputs_.tiltAbsoluteEncoderPositionMedian ;
            encoders_synced_ = true ;
            synced = true ;
        }

        double tiltdiff = Math.abs(inputs_.tiltAbsoluteEncoderPosition - motortilt) ;
        if (inputs_.tiltAbsoluteEncoderPositionMedian < IntakeShooterConstants.Tilt.Resync.kPosThreshold &&
            Math.abs(average_value_) < IntakeShooterConstants.Tilt.Resync.kVelThreshold && state_ == State.Idle &&
            encoders_synced_ && tiltdiff > IntakeShooterConstants.Tilt.Resync.kPosDiffThreshold) { 
            if (!XeroRobot.isSimulation()) {
                syncTiltEncoders(false) ;
                synced = true ;
            }
        }

        Logger.recordOutput("intake:synced", synced) ;
        Logger.processInputs("intake-shooter", inputs_);

        if (average_filter_ != null) {
            double now = Timer.getFPGATimestamp() ;
            double vel = (inputs_.tiltAbsoluteEncoderPositionMedian - last_value_) / (now - last_time_) ;
            average_value_ = average_filter_.calculate(vel) ;

            Logger.recordOutput("intake:tilt-abs-velocity", average_value_) ;
            Logger.recordOutput("intake:tilt-raw-vel", vel) ;

            last_time_ = now ;
            last_value_ = inputs_.tiltAbsoluteEncoderPositionMedian ;
        }

        NoteDestination dest = getNoteDestination() ;
        if (tracking_ && (dest == NoteDestination.Amp || dest == NoteDestination.Trap)) {
            //
            // We are transitioning from shoot to Amp or Trap, turn off tracking
            // and stop the shooter wheels
            //
            setTracking(false) ;
            io_.setShooter1MotorVoltage(0.0) ;
            io_.setShooter2MotorVoltage(0.0) ;
            moveToTransferPosition() ;
        }

        switch(state_) {
            case Idle:
            case Invalid:
                break ;

            case TransferringNote:
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

            case StartEjectOperations:
                setShooterVelocity(-IntakeShooterConstants.Shooter.kEjectVelocity, 10.0) ;
                io_.setFeederMotorVoltage(-IntakeShooterConstants.Feeder.kEjectVoltage) ;
                eject_forward_timer_.start() ;
                has_note_ = false ;
                state_ = State.EjectForward ;
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

            case TransferFinish1:
                if (inputs_.shooter1Position - transfer_start_pos_ > IntakeShooterConstants.Shooter.kTransferLength) {
                    io_.setShooter1MotorVoltage(0.0) ;
                    io_.setShooter2MotorVoltage(0.0);
                    io_.setFeederMotorVoltage(0.0);
                    gotoPosition(State.TransferFinish2,
                         IntakeShooterConstants.UpDown.Positions.kFinishTransfer, 
                         IntakeShooterConstants.Tilt.Positions.kFinishTransfer) ;
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
            oi.setLEDState(OILed.ShooterReady, isShooterReady() ? LEDState.On : LEDState.Off) ;
            oi.setLEDState(OILed.TiltReady, isTiltReady() ? LEDState.On : LEDState.Off) ;
        }

        visualizer_.updateIntakeShooter(
            Degrees.of(-inputs_.updownPosition),
            Degrees.of(-inputs_.tiltAbsoluteEncoderPosition)
        );

        noteVisualizer_.updateIntake(has_note_);

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

            Logger.recordOutput("intake:cappos", io_.getShooterPositionAtRisingEdge());

            Logger.recordOutput("intake:firing", firing_);
        }        

        endPeriodic();
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

