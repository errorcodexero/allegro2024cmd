package frc.robot.subsystems;

import org.xero1425.PieceWiseLinear;
import org.xero1425.TalonFXFactory;
import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;
import org.xero1425.XeroTimer;

import java.util.function.DoubleSupplier;
import org.xero1425.EncoderMapper;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CollectCommand;
import frc.robot.constants.IntakeShooterConstants;

public class IntakeShooterSubsystem extends XeroSubsystem {

    public enum NoteDestination {
        Speaker,
        TrapAmp
    }
    
    private enum State {
        Idle,
        MoveTiltToPosition,
        MoveBothToPosition,
        WaitForNote,
        WaitForCapture,
        WaitForReverse,
        HoldingTransferPosition,
        TransferStartedFeeder,
        TransferringNote,
        WaitingToShoot,
        WaitForShotFinish,
        EjectForward,
        EjectPause,
        EjectReverse
    }

    private TalonFX feeder_motor_ ;
    private TalonFX updown_motor_ ;
    private TalonFX shooter1_motor_ ;
    private TalonFX shooter2_motor_ ;
    private TalonFX tilt_motor_ ;
    private DigitalInput note_sensor_ ;    
    private AnalogInput absolute_encoder_;
    
    private EncoderMapper encoder_mapper_;    

    private double target_tilt_ ;
    private double target_tilt_tol_ ;
    private double target_tilt_vel_ ;
    private double next_tilt_ ;
    private double target_updown_ ;
    private double target_updown_tol_ ;
    private double target_updown_vel_ ;
    private double target_velocity_ ;
    private double next_updown_ ;
    private NoteDestination destination_ ;

    private boolean tracking_ ;
    private DoubleSupplier distsupplier_ ;
    private PieceWiseLinear updown_pwl_ ;
    private PieceWiseLinear tilt_pwl_ ;
    private PieceWiseLinear velocity_pwl_ ;

    private AsynchronousInterrupt note_interrupt_ ;
    private boolean has_note_ ;
    private boolean rising_seen_ ;
    private boolean falling_seen_ ;

    private XeroTimer capture_timer_ ;
    private XeroTimer reverse_timer_ ;
    private XeroTimer transfer_timer_ ;
    private XeroTimer shoot_timer_ ;
    private XeroTimer eject_forward_timer_ ;
    private XeroTimer eject_reverse_timer_ ;
    private XeroTimer eject_pause_timer_ ;

    private State state_ ;
    private State next_state_ ;

    private DCMotorSim updown_sim_ ;
    private DCMotorSim tilt_sim_ ;

    public IntakeShooterSubsystem(XeroRobot robot, DoubleSupplier distsupplier) throws Exception {
        super(robot, "intake-shooter") ;

        Slot0Configs cfg ;

        feeder_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Feeder.kMotorId,
                    false,
                    IntakeShooterConstants.Feeder.kCurrentLimit);

        updown_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.UpDown.kMotorId,
                    false,
                    IntakeShooterConstants.UpDown.kCurrentLimit);
        updown_motor_.getPosition().setUpdateFrequency(100) ;
        updown_motor_.getVelocity().setUpdateFrequency(100) ;
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.UpDown.PID.kP)
                                .withKI(IntakeShooterConstants.UpDown.PID.kI)
                                .withKD(IntakeShooterConstants.UpDown.PID.kD)
                                .withKV(IntakeShooterConstants.UpDown.PID.kV)
                                .withKA(IntakeShooterConstants.UpDown.PID.kA)
                                .withKG(IntakeShooterConstants.UpDown.PID.kG)
                                .withKS(IntakeShooterConstants.UpDown.PID.kS) ;
        updown_motor_.getConfigurator().apply(cfg) ;

                    
        tilt_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Tilt.kMotorId,
                    false,
                    IntakeShooterConstants.Tilt.kCurrentLimit);
        tilt_motor_.getPosition().setUpdateFrequency(100) ;
        tilt_motor_.getVelocity().setUpdateFrequency(100) ;        
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.Tilt.PID.kP)
                                .withKI(IntakeShooterConstants.Tilt.PID.kI)
                                .withKD(IntakeShooterConstants.Tilt.PID.kD)
                                .withKV(IntakeShooterConstants.Tilt.PID.kV)
                                .withKA(IntakeShooterConstants.Tilt.PID.kA)
                                .withKG(IntakeShooterConstants.Tilt.PID.kG)
                                .withKS(IntakeShooterConstants.Tilt.PID.kS) ;
        tilt_motor_.getConfigurator().apply(cfg) ;

        shooter1_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter1.kMotorId,
                    false,
                    IntakeShooterConstants.Shooter1.kCurrentLimit);
        shooter1_motor_.getVelocity().setUpdateFrequency(100) ;
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.Shooter.kP)
                                .withKI(IntakeShooterConstants.Shooter.kI)
                                .withKD(IntakeShooterConstants.Shooter.kD)
                                .withKV(IntakeShooterConstants.Shooter.kV)
                                .withKA(IntakeShooterConstants.Shooter.kA)
                                .withKG(IntakeShooterConstants.Shooter.kG)
                                .withKS(IntakeShooterConstants.Shooter.kS) ;
        shooter1_motor_.getConfigurator().apply(cfg) ;

        shooter2_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter2.kMotorId,
                    false,
                    IntakeShooterConstants.Shooter2.kCurrentLimit);
        shooter2_motor_.getVelocity().setUpdateFrequency(100) ;                    
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.Shooter.kP)
                                .withKI(IntakeShooterConstants.Shooter.kI)
                                .withKD(IntakeShooterConstants.Shooter.kD)
                                .withKV(IntakeShooterConstants.Shooter.kV)
                                .withKA(IntakeShooterConstants.Shooter.kA)
                                .withKG(IntakeShooterConstants.Shooter.kG)
                                .withKS(IntakeShooterConstants.Shooter.kS) ;
        shooter2_motor_.getConfigurator().apply(cfg) ;

        note_sensor_ = new DigitalInput(IntakeShooterConstants.NoteSensor.kChannel) ;
        note_interrupt_ = new AsynchronousInterrupt(note_sensor_, (rising, falling) -> { noteInterruptHandler(rising, falling); }) ;
        note_interrupt_.setInterruptEdges(true, true);            
        note_interrupt_.enable();

        absolute_encoder_ = new AnalogInput(IntakeShooterConstants.Tilt.AbsoluteEncoder.kChannel) ;
        encoder_mapper_ = new EncoderMapper(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMin,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMin) ;

        if (getRobot().isPracticeBot()) {
            encoder_mapper_.calibrate(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotCalibrationValue,
                                      IntakeShooterConstants.Tilt.AbsoluteEncoder.Practice.kEncoderCalibrationValue) ;
        }
        else {
            encoder_mapper_.calibrate(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotCalibrationValue,
                                      IntakeShooterConstants.Tilt.AbsoluteEncoder.Competition.kEncoderCalibrationValue) ;            
        }

        capture_timer_ = new XeroTimer(getRobot(), "collect-timer", IntakeShooterConstants.kCollectDelayTime) ;
        reverse_timer_ = new XeroTimer(getRobot(), "reverse-timer", IntakeShooterConstants.kReverseDelayTime) ;
        transfer_timer_ = new XeroTimer(getRobot(), "transfer-timer", IntakeShooterConstants.kTransferFeederToShooterDelay);
        shoot_timer_ = new XeroTimer(getRobot(), "shoot-timer", IntakeShooterConstants.Feeder.kShootTime) ;

        setTiltMotorPosition(getTiltAbsoluteEncoderPosition());
        setUpDownMotorPosition(IntakeShooterConstants.UpDown.Positions.kStowed);

        tracking_ = false ;
        distsupplier_ = distsupplier ;
        updown_pwl_ = new PieceWiseLinear(IntakeShooterConstants.UpDown.kPwlValues) ;
        tilt_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Tilt.kPwlValues) ;
        velocity_pwl_ = new PieceWiseLinear(IntakeShooterConstants.Shooter.kPwlValues) ;

        eject_forward_timer_ = new XeroTimer(getRobot(), "eject-1", IntakeShooterConstants.Shooter.kEjectForwardTime) ;
        eject_reverse_timer_ = new XeroTimer(getRobot(), "eject-1", IntakeShooterConstants.Shooter.kEjectReverseTime) ;
        eject_pause_timer_ = new XeroTimer(getRobot(), "eject-1", IntakeShooterConstants.Shooter.kEjectPauseTime) ;

        state_ = State.Idle ;

        if (RobotBase.isSimulation()) {
            updown_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 49.5, 0.0000001) ;
            tilt_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 18.0, 0.0000001) ;
        }

        destination_ = NoteDestination.Speaker ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Subsystem command factor interface
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    public Command collectCommand() {
        return new CollectCommand(getRobot()) ;
    }

    public Command ejectCommand() {
        return runOnce(this::eject) ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // State about the subsystem
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public boolean isIdle() {
        return state_ == State.Idle ;
    }

    public double getUpDownMotorPosition() {
        return updown_motor_.getPosition().getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
    }

    public void setUpDownMotorPosition(double pos) {
        updown_motor_.setPosition(pos / IntakeShooterConstants.UpDown.kDegreesPerRev) ;
    }

    public double getUpDownMotorVelocity() {
        return updown_motor_.getVelocity().getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
    }   

    public double getTiltMotorPosition() {
        return tilt_motor_.getPosition().getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
    }

    public void setTiltMotorPosition(double pos) {
        tilt_motor_.setPosition(pos / IntakeShooterConstants.Tilt.kDegreesPerRev) ;
    }

    public double getTiltMotorVelocity() {
        return tilt_motor_.getVelocity().getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
    }   

    public double getTiltAbsoluteEncoderPosition() {
        return encoder_mapper_.toRobot(absolute_encoder_.getVoltage()) ;
    }

    public double getShooter1Velocity() {
        return shooter1_motor_.getVelocity().getValueAsDouble() ;
    }

    public double getShooter2Velocity() {
        return shooter2_motor_.getVelocity().getValueAsDouble() ;
    }

    public boolean isInTransferPosition() {
        return state_ == State.HoldingTransferPosition ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Methods to set hardwrae targets
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

    public void setShooterVelocity(double vel) {
        shooter1_motor_.setControl(new VelocityTorqueCurrentFOC(vel)) ;
        shooter2_motor_.setControl(new VelocityTorqueCurrentFOC(vel)) ;     
        target_velocity_ = vel ;           
    }

    public void setShooterVoltage(double v) {
        shooter1_motor_.setControl(new VoltageOut(v)) ;
        shooter2_motor_.setControl(new VoltageOut(v)) ;
        target_velocity_ = Double.NaN ;
    }

    public void setUpDownTarget(double t) {
        updown_motor_.setControl(new PositionTorqueCurrentFOC(t / IntakeShooterConstants.UpDown.kDegreesPerRev)) ;
        target_updown_ = t ;
    }

    public void setUpDownVolage(double v) {
        updown_motor_.setControl(new VoltageOut(v)) ;
        target_updown_ = Double.NaN ;
    }

    public void setTiltTarget(double t) {
        tilt_motor_.setControl(new PositionTorqueCurrentFOC(t / IntakeShooterConstants.Tilt.kDegreesPerRev)) ;
        target_tilt_ = t ;
    }

    public void setTiltVoltage(double v) {
        tilt_motor_.setControl(new VoltageOut(v)) ;
        target_tilt_ = Double.NaN ;
    }

    public void setFeederVoltage(double v) {
        feeder_motor_.setControl(new VoltageOut(v)) ;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Methods that cause the subsystem to take action
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void collect() {
        if (!has_note_ && state_ == State.Idle) {
            //
            // Turn on the feeder
            //
            setFeederVoltage(IntakeShooterConstants.Feeder.kCollectVoltage);

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

    public void stopCollect() {
        if (isMoving()) {
            //
            // Turn off the feeder
            //
            setFeederVoltage(0.0);
            setTiltTarget(getTiltMotorPosition());
            setUpDownTarget(getUpDownMotorPosition());

            state_ = State.WaitForReverse ;
            
        }
        else if (state_ == State.WaitForNote) {
            //
            // Turn off the feeder
            //
            setFeederVoltage(0.0);
            
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

    public void startTransferNote() {
        if (state_ == State.HoldingTransferPosition) {
            //
            // Turn on the feeder first
            //
            setFeederVoltage(IntakeShooterConstants.Feeder.kTransferVoltage);
            transfer_timer_.start();
            state_ = State.TransferStartedFeeder ;
        }
    }

    public void stopTransferNote() {
        //
        // Turn off the feeder
        //
        setFeederVoltage(0.0);
        setShooterVoltage(0.0);

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

    public void manualShoot(double updown, double tilt, double velocity) {
        gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;
        setShooterVelocity(velocity) ;

        state_ = State.WaitingToShoot ;

    }

    public void startAutoShoot() {
        if (has_note_ && state_ == State.Idle) {
            tracking_ = true ;
        }
    }

    public void cancelAutoShoot() {
        tracking_ = false ;
        setShooterVoltage(0.0) ;

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

    public void finishShot() {
        tracking_ = false ;
        state_ = State.WaitingToShoot ;
    }

    public void eject() {
        setShooterVoltage(IntakeShooterConstants.Shooter.kEjectVoltage) ;
        eject_forward_timer_.start() ;
        state_ = State.EjectForward ;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Implementation
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

            next_updown_ = Double.NaN ;
            next_tilt_ = Double.NaN ;

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
        if (rising_seen_ || falling_seen_) {
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
            setFeederVoltage(0.0);

            //
            // Move the updown and tilt to the stowed position
            //
            double updown = IntakeShooterConstants.UpDown.Positions.kStowed ;
            double tilt = IntakeShooterConstants.Tilt.Positions.kStowed ;
            if (has_note_) {
                switch(destination_) {
                    case Speaker:
                        tracking_ = true ;
                        break ;

                    case TrapAmp:
                        updown = IntakeShooterConstants.UpDown.Positions.kTransfer ;
                        tilt = IntakeShooterConstants.Tilt.Positions.kTransfer ;
                        break ;
                }
                updown = IntakeShooterConstants.UpDown.Positions.kTransfer ;
                tilt = IntakeShooterConstants.Tilt.Positions.kTransfer ;
            }
            gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;

            //
            // And set the state for after we reach the stowed position
            //
            next_state_ = State.Idle ;
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

    private void transferStartedFeederState() {
        if (transfer_timer_.isExpired()) {
            //
            // Turn on the shooter wheels
            //
            setShooterVelocity(IntakeShooterConstants.kTransferShooterVelocity) ;
            state_ = State.TransferringNote ;
        }
    }

    private boolean isTiltReady() {
        return 
            Math.abs(getTiltMotorPosition() - target_tilt_) < target_tilt_tol_ &&
            Math.abs(getTiltMotorVelocity()) < target_tilt_vel_ ;
    }

    private boolean isUpDownReady() {
        return 
            Math.abs(getUpDownMotorPosition() - target_updown_) < target_updown_tol_ &&
            Math.abs(getUpDownMotorVelocity()) < target_updown_vel_ ;
    }

    private boolean isShooterReady() {
        return 
            Math.abs(getShooter1Velocity() - target_velocity_) < IntakeShooterConstants.kShooterVelocityTolerance &&
            Math.abs(getShooter2Velocity() - target_velocity_) < IntakeShooterConstants.kShooterVelocityTolerance ;
    }

    private void waitingToShootState() {
        if (isTiltReady() && isUpDownReady() && isShooterReady()) {
            setFeederVoltage(IntakeShooterConstants.Feeder.kShootVoltage) ;
            shoot_timer_.start() ;
            state_ = State.WaitForShotFinish ;
        }
    }

    private void waitForShotFinishState() {
        if (shoot_timer_.isExpired()) {
            //
            // Turn off the feeder
            //
            setFeederVoltage(0.0);

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
            setFeederVoltage(0.0);
            eject_pause_timer_.start() ;
            state_ = State.EjectPause ;
        }
    }

    private void ejectPauseState() {
        if (eject_pause_timer_.isExpired()) {
            setShooterVoltage(-IntakeShooterConstants.Shooter.kEjectVoltage) ;
            setFeederVoltage(-IntakeShooterConstants.Feeder.kEjectVoltage) ;
            eject_reverse_timer_.start() ;
            state_ = State.EjectReverse ;
        }
    }

    private void ejectReverseState() {
        if (eject_reverse_timer_.isExpired()) {
            setShooterVoltage(0.0);
            setFeederVoltage(0.0);
            state_ = State.Idle ;
        }
    }

    private void trackTargetDistance() {
        double dist = distsupplier_.getAsDouble() ;
        double updown = updown_pwl_.getValue(dist) ;
        double tilt = tilt_pwl_.getValue(dist) ;
        double velocity = velocity_pwl_.getValue(dist) ;

        gotoPosition(updown, Double.NaN, Double.NaN, tilt, Double.NaN, Double.NaN) ;
        setShooterVelocity(velocity) ;
    }

    private void logData() {
        SmartDashboard.putNumber("updown-pos", getUpDownMotorPosition()) ;
        SmartDashboard.putNumber("updown-vel", getUpDownMotorVelocity()) ;
        SmartDashboard.putNumber("updown-target", target_updown_) ;
        SmartDashboard.putNumber("tilt-abspos", getTiltAbsoluteEncoderPosition()) ;
        SmartDashboard.putNumber("tilt-pos", getTiltMotorPosition()) ;
        SmartDashboard.putNumber("tilt-vel", getTiltMotorVelocity()) ;
        SmartDashboard.putNumber("tilt-target", target_tilt_) ;
        SmartDashboard.putNumber("shooter1-vel", getShooter1Velocity()) ;
        SmartDashboard.putNumber("shooter2-vel", getShooter2Velocity()) ;
    }

    public void periodic() {
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

            case TransferStartedFeeder:
                transferStartedFeederState() ;
                break ;

            case TransferringNote:
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
        }

        if (tracking_ && distsupplier_ != null) {
            trackTargetDistance() ;
        }

        rising_seen_ = false ;
        falling_seen_ = false ;

        logData() ;
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

        double desired_tilt = computeTiltFromUpdown(updown) ;
        if (Math.abs(getTiltMotorPosition() - desired_tilt) > IntakeShooterConstants.Tilt.kAllowedDeviationFromTrack) {
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

    private void noteInterruptHandler(boolean rising, boolean falling) {
        rising_seen_ = rising ;
        falling_seen_ =  falling ;
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState state ;

        state = tilt_motor_.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        SmartDashboard.putNumber("tilt-voltage", state.getMotorVoltage()) ;
        
        tilt_sim_.setInputVoltage(state.getMotorVoltage());
        tilt_sim_.update(getRobot().getPeriod());
        state.setRawRotorPosition(tilt_sim_.getAngularPositionRotations()) ;
        state.setRotorVelocity(Units.radiansToRotations(tilt_sim_.getAngularVelocityRadPerSec())) ;

        state = updown_motor_.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;
        updown_sim_.setInputVoltage(state.getMotorVoltage());
        updown_sim_.update(getRobot().getPeriod());
        state.setRawRotorPosition(updown_sim_.getAngularPositionRotations()) ;
        state.setRotorVelocity(Units.radiansToRotations(updown_sim_.getAngularVelocityRadPerSec())) ;        
    }
}
