package frc.robot.subsystems.intakeshooter ;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.base.XeroTimer;
import org.xero1425.math.PieceWiseLinear;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units ;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.NoteDestination;
import frc.robot.ShotType;

public class IntakeShooterSubsystem extends XeroSubsystem {
    static final public String NAME = "IntakeShooterSubsystem" ;
    static final public String FEEDER_MOTOR_NAME = "feeder" ;
    static final public String SHOOTER1_MOTOR_NAME = "shooter1" ;
    static final public String SHOOTER2_MOTOR_NAME = "shooter2" ;
    static final public String UPDOWN_MOTOR_NAME = "updown" ;
    static final public String TILT_MOTOR_NAME = "tilt" ;        
    
   
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
        EjectForward,
        EjectPause,
        EjectReverse,
        TransferStartingShooter,
        TransferWaitForNote,
        TransferWaitForNoNote,
        TransferFinishTransfer,
        TransferContinueShooter,
        TransferRunShooter,
        TransferRunShooterWaitForStop,
        EnableTracking,
        HoldForShoot,
        GoToEjectPosition,
        Tuning,
        WaitingForTunedNoteShot1,
        WaitingForTunedNoteShot2,        
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

    private boolean has_note_ ;
    private boolean need_stop_manipulator_ ;

    private XeroTimer capture_timer_ ;
    private XeroTimer reverse_timer_ ;
    private XeroTimer shoot_timer_ ;
    private XeroTimer eject_forward_timer_ ;
    private XeroTimer eject_reverse_timer_ ;
    private XeroTimer eject_pause_timer_ ;
    private XeroTimer transfer_shooter_timer_ ;

    private State state_ ;
    private State next_state_ ;
    private State initial_transfer_state_ ;

    private Supplier<NoteDestination> destsupplier_ ;
    private Supplier<ShotType> shot_type_supplier_ ;

    private Trigger transfer_note_trigger_ ;
    private Trigger ready_for_shoot_trigger_ ;
    private boolean collect_after_manual_ ;

    public IntakeShooterSubsystem(XeroRobot robot, DoubleSupplier distsupplier, Supplier<NoteDestination> destsupplier, Supplier<ShotType> shottype) throws Exception {
        super(robot, NAME) ;

        io_ = new IntakeShooterIOHardware(robot) ;
        inputs_ = new IntakeShooterIOInputsAutoLogged() ;

        destsupplier_ = destsupplier ;
        shot_type_supplier_ = shottype ;

        capture_timer_ = new XeroTimer("collect-timer", IntakeShooterConstants.kCollectDelayTime) ;
        reverse_timer_ = new XeroTimer("reverse-timer", IntakeShooterConstants.kReverseDelayTime) ;
        shoot_timer_ = new XeroTimer("shoot-timer", IntakeShooterConstants.Feeder.kShootTime) ;
        eject_forward_timer_ = new XeroTimer("eject-forward", IntakeShooterConstants.Shooter.kEjectForwardTime) ;
        eject_reverse_timer_ = new XeroTimer("eject-reverse", IntakeShooterConstants.Shooter.kEjectReverseTime) ;
        eject_pause_timer_ = new XeroTimer("eject-pause", IntakeShooterConstants.Shooter.kEjectPauseTime) ;
        transfer_shooter_timer_ = new XeroTimer("transfer-shooter", IntakeShooterConstants.Shooter.kTransferRunShooterDuration) ;

        io_.setTiltMotorPosition(io_.getTiltAbsoluteEncoderPosition());
        io_.setUpDownMotorPosition(IntakeShooterConstants.UpDown.Positions.kStowed);

        setTracking(false) ;
        distsupplier_ = distsupplier ;
        updown_pwl_ = new PieceWiseLinear(IntakeShooterConstants.UpDown.kPwlValues) ;
        tilt_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Tilt.kPwlValues) ;
        velocity_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Shooter.kPwlValues) ;

        state_ = State.Idle ;
        next_state_ = State.Invalid ;

        transfer_note_trigger_ = new Trigger(()-> transferNote()) ;
        ready_for_shoot_trigger_ = new Trigger(()-> state_ == State.HoldForShoot) ;

        need_stop_manipulator_ = false ;
    }

    public String stateString() {
        return state_.toString() ;
    }

    public void startTuning() {
        if (state_ == State.Idle) {
            state_ = State.Tuning ;
        }
    }

    public Map<String, TalonFX> getCTREMotors() {
        return io_.getCTREMotors() ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

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

    public Trigger readyForShoot() {
        return ready_for_shoot_trigger_ ;
    }

    public Trigger readyForTransferNote() {
        return transfer_note_trigger_ ;
    }

    public boolean hasNote() {
        return has_note_ ;
    }

    public boolean transferNote() {
        boolean ret = false ;

        if (!DriverStation.isAutonomous()) {
            NoteDestination dest = getNoteDestination() ;
            ret = has_note_ && 
                     (dest == NoteDestination.Trap ||  dest == NoteDestination.Amp) &&
                     (state_ == State.Idle || state_ == State.MoveTiltToPosition || state_ == State.MoveBothToPosition || state_ == State.HoldForShoot) ;
        }
        return ret;
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
            () -> isIdle());
        ret.setName("manual-shoot");
        return ret ;
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

    public boolean isTuning() {
        return state_ == State.Tuning ;
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

    public void setManualShootParameters(double updown, double tilt) {
        auto_manual_shoot_tilt_ = tilt ;
        auto_manual_shoot_updown_ = updown ;
    }

    //
    // If we are collecting, this stops the collect operation.
    //
    public void stopCollect() {
        if (isMoving() || state_ == State.WaitForNote) {
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

    //
    // If the intake has a note, this method transfers the note to the manipulator.  This
    // method assumes the intake/shooter and elevator/manipulator are in the correct positions
    // before being called.  If they are in the transfer position, the state will be HoldingTransferPosition.
    //
    public void transferNoTrampSensor(int strategy) {
        if (has_note_ && state_ == State.HoldingTransferPosition) {
            need_stop_manipulator_ = false ;

            if (strategy == 1) {
                transfer_shooter_timer_.start() ;                
                initial_transfer_state_ = State.TransferRunShooter;
            }
            else {
                if (isNoteDetected()) {
                    //
                    // The note is already sitting on the sensor.  We just wait for the note to
                    // move off the sensor
                    //
                    initial_transfer_state_ = State.TransferWaitForNoNote ;
                }
                else {
                    //
                    // The note is not on the sensor, so the sensor is assumed to be sitting in the middle of the note.
                    // We wait until we sense the note and then look for the note to move off the sensor.
                    //
                    initial_transfer_state_ = State.TransferWaitForNote ;
                }
            }

            //
            // Start the shooter wheels so they are moving when the note hits the shooter.
            //
            setShooterVelocity(IntakeShooterConstants.Shooter.kTransferVelocity, IntakeShooterConstants.Shooter.kTransferVelocityTol) ;
            state_ = State.TransferStartingShooter ;            
        }
    }

    public void transferWithTrampSensor() {
        if (has_note_ && state_ == State.HoldingTransferPosition) {      
            //
            // Start the shooter wheels so they are moving when the note hits the shooter.
            //
            setShooterVelocity(IntakeShooterConstants.Shooter.kTransferVelocity, IntakeShooterConstants.Shooter.kTransferVelocityTol) ;
            initial_transfer_state_ = State.TransferRunShooterWaitForStop ;
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

    public void tiltToTest(int angle) {
        setTiltTarget(angle);
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

    public void abortShot() {
        io_.setShooter1MotorVoltage(0.0);
        io_.setShooter2MotorVoltage(0.0);
    }

    public void eject() {
        setTracking(false);
        gotoPosition(IntakeShooterConstants.UpDown.Positions.kEject, 5.0, 100.0, 
                     IntakeShooterConstants.Tilt.Positions.kEject, 8.0, 100.0) ;
        state_ = State.GoToEjectPosition ;
    }

    public void setHasNote(boolean b) {
        has_note_ = b ;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Implementation
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private NoteDestination getNoteDestination() {
        NoteDestination ret = NoteDestination.Speaker ;
        if (DriverStation.isAutonomous()) {
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

    public void setUpDownTarget(double pos) {
        target_updown_ = pos ;
        io_.setUpDownTargetPos(pos);
    }

    public void setTiltTarget(double pos) {
        io_.setTiltTargetPos(tracking_, pos);
        target_tilt_ = pos ;
    }

    public void setShooterVelocity(double vel, double veltol) {
        io_.setShooter1Velocity(vel);
        io_.setShooter2Velocity(vel);

        target_velocity_tol_ = veltol ;
        target_velocity_ = vel ;
    }

    private void setShooterVoltage(double v) {
        target_velocity_ = 0 ;
        io_.setShooter1MotorVoltage(v);
        io_.setShooter2MotorVoltage(v);
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
            next_state_ = State.Invalid ;
        }
        else if (next_state_ == State.WaitForNote && inputs_.fallingEdge) {
            has_note_ = true ;
            capture_timer_.start() ;
            state_ = State.WaitForCapture ;            
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
                            next_state_ = State.EnableTracking ;
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

    public void stopShooterInTransfer() {
        target_velocity_ = 0 ;
        has_note_ = false ;
        setShooterVoltage(0.0);
        io_.setFeederMotorVoltage(0.0);

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

    public boolean hasShotLeft() {
        return state_ != State.WaitForShotFinish ;
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

    public boolean isTiltReady() {
        boolean b =  
            Math.abs(inputs_.tiltPosition - target_tilt_) < target_tilt_tol_ &&
            Math.abs(inputs_.tiltVelocity) < target_tilt_vel_ ;
        return b ;
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

    public void startFeeder() {
        io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kShootVoltage) ;
        state_ = State.WaitingForTunedNoteShot1 ;
    }

    public void stopFeeder() {
        io_.setFeederMotorVoltage(0.0) ;
        state_ = State.Tuning ;
    }

    public void stopShooter() {
        io_.setShooter1MotorVoltage(0.0) ;
        io_.setShooter2MotorVoltage(0.0) ;
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
            // Turn off the feeder
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
            setShooterVoltage(0.0);
            io_.setFeederMotorVoltage(0.0) ;
            eject_pause_timer_.start() ;
            state_ = State.EjectPause ;
        }
    }

    private void ejectPauseState() {
        if (eject_pause_timer_.isExpired()) {
            setShooterVoltage(-IntakeShooterConstants.Shooter.kEjectVoltage) ;
            io_.setFeederMotorVoltage(-IntakeShooterConstants.Feeder.kEjectVoltage) ;
            eject_reverse_timer_.start() ;
            state_ = State.EjectReverse ;
        }
    }

    private void ejectReverseState() {
        if (eject_reverse_timer_.isExpired()) {
            setShooterVoltage(0.0);
            io_.setFeederMotorVoltage(0.0);
            state_ = State.Idle ;
        }
    }

    private void transferStartingShooterState() {
        if (isShooterReady()) {
            io_.setFeederMotorVoltage(IntakeShooterConstants.Feeder.kTransferVoltage);
            state_ = initial_transfer_state_ ;
        }
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

    @Override
    public void simulationPeriodic() {
        io_.simulate(getRobot().getPeriod()) ;
    }

    @Override
    public void periodic() {
        periodicStart();

        io_.updateInputs(inputs_);
        Logger.processInputs("intake-shooter", inputs_);

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
                transferStartingShooterState() ;
                break ;

            case TransferWaitForNote:
                if (inputs_.risingEdge && inputs_.fallingEdge) {
                    //
                    // The note completely passed the sensor since the last robot loop
                    //
                    state_ = State.TransferFinishTransfer ;
                    transfer_start_pos_ = inputs_.shooter1Position ;                    
                }
                else if (inputs_.fallingEdge) {
                    //
                    // The leading edge of the note passed the sensor, wait for the trailing edge
                    //
                    state_ = State.TransferWaitForNoNote ;
                }
                break ;

            case TransferRunShooterWaitForStop:
                break ;

            case TransferWaitForNoNote:
                if (inputs_.risingEdge) {
                    state_ = State.TransferFinishTransfer ;
                    transfer_start_pos_ = inputs_.shooter1Position ;
                }
                break ;

            case TransferFinishTransfer:
                if (inputs_.shooter1Position - transfer_start_pos_ > IntakeShooterConstants.Shooter.kTransferLength) {                
                    need_stop_manipulator_ = true ;
                    has_note_ = false ;                    
                    state_ = State.TransferContinueShooter ;
                }
                break ;

            case TransferContinueShooter:
                if (inputs_.shooter1Position - transfer_start_pos_ - IntakeShooterConstants.Shooter.kTransferLength > IntakeShooterConstants.Shooter.kTransferContLength) {
                    io_.setFeederMotorVoltage(0.0);
                    setShooterVoltage(0.0);
                    next_state_ = State.Idle ;                    
                    gotoPosition(IntakeShooterConstants.UpDown.Positions.kStowed, Double.NaN, Double.NaN,
                                 IntakeShooterConstants.Tilt.Positions.kStowed, Double.NaN, Double.NaN) ;
                }
                break ;

            case TransferRunShooter:
                if (transfer_shooter_timer_.isExpired()) {
                    io_.setFeederMotorVoltage(0.0);
                    setShooterVoltage(0.0);
                    state_ = State.Idle ;                    
                }
                break ;

            case EnableTracking:
                setTracking(true);
                state_ = State.HoldForShoot ;
                break ;

            case GoToEjectPosition:
                if (isTiltReady() && isUpDownReady()) {
                    setShooterVoltage(IntakeShooterConstants.Shooter.kEjectVoltage);
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

        if (getVerbose()) {
            Logger.recordOutput("intake:state", ststr);
            Logger.recordOutput("intake:next-state", next_state_) ;
            Logger.recordOutput("intake:updown-target", target_updown_) ;
            Logger.recordOutput("intake:updown-vel", inputs_.updownVelocity) ;
            Logger.recordOutput("intake:tilt-target", target_tilt_) ;
            Logger.recordOutput("intake:shooter-target", target_velocity_) ;
            Logger.recordOutput("intake:is-tilt-ready", isTiltReady());
            Logger.recordOutput("intake:is-updown-ready", isUpDownReady());
            Logger.recordOutput("intake:is-shooter-ready", isShooterReady());
            Logger.recordOutput("intake:has-note", has_note_);
            Logger.recordOutput("intake:tracking", tracking_);
            Logger.recordOutput("intake:note-dest", getNoteDestination()) ;
            Logger.recordOutput("intake:shot-type", getShotType()) ;
            Logger.recordOutput("intake:feederVoltage", io_.getFeederMotorVoltage()) ;
            Logger.recordOutput("intake:readyForShoot", readyForShoot().getAsBoolean()) ;
            Logger.recordOutput("intake:tracking", tracking_) ;
            Logger.recordOutput("intake:needStopManip", need_stop_manipulator_);
        }
        periodicEnd();
    }

    private void setTracking(boolean b) {
        try {
            tracking_ = b ;
            io_.setTiltMovementPID();
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception setting tracking mode - ").add(ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }
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
}

