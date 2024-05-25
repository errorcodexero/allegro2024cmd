package frc.robot.subsystems.intakeshooter;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import org.xero1425.EncoderMapper;
import org.xero1425.TalonFXFactory;
import org.xero1425.XeroRobot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.units.Units ;

public class IntakeShooterIOHardware implements IntakeShooterIO {

    private final static int kApplyTries = 5 ;  

    private TalonFX feeder_motor_ ;
    private TalonFX updown_motor_ ;
    private TalonFX shooter1_motor_ ;
    private TalonFX shooter2_motor_ ;
    private TalonFX tilt_motor_ ;
    private DigitalInput note_sensor_ ;    
    private AnalogInput absolute_encoder_;
    private AsynchronousInterrupt note_interrupt_ ;
    private EncoderMapper encoder_mapper_ ;
    private AtomicBoolean rising_seen_ ;
    private AtomicBoolean falling_seen_ ;

    private DCMotorSim updown_sim_ ;
    private DCMotorSim tilt_sim_ ;
    private DCMotorSim shooter1_sim_ ;
    private DCMotorSim shooter2_sim_ ;

    private DIOSim note_sim_ ;
    private boolean is_sim_ ;

    private double updown_voltage_ ;
    private double tilt_voltage_ ;
    private double shooter1_voltage_ ;
    private double shooter2_voltage_ ;
    private double feeder_voltage_ ;

    private StatusSignal<Double> updown_position_signal_ ;
    private StatusSignal<Double> updown_velocity_signal_ ;
    private StatusSignal<Double> updown_current_signal_ ;
    private StatusSignal<Double> updown_voltage_signal_ ;    

    private StatusSignal<Double> tilt_position_signal_ ;
    private StatusSignal<Double> tilt_velocity_signal_ ;
    private StatusSignal<Double> tilt_current_signal_ ;
    private StatusSignal<Double> tilt_voltage_signal_ ;

    private StatusSignal<Double> feeder_current_signal_ ;
    private StatusSignal<Double> shooter1_velocity_signal_ ;
    private StatusSignal<Double> shooter1_current_signal_ ;
    private StatusSignal<Double> shooter1_position_signal_ ;
    private StatusSignal<Double> shooter1_voltage_signal_ ;

    private StatusSignal<Double> shooter2_velocity_signal_ ;
    private StatusSignal<Double> shooter2_current_signal_ ;
    private StatusSignal<Double> shooter2_position_signal_ ;
    private StatusSignal<Double> shooter2_voltage_signal_ ;

    public IntakeShooterIOHardware(XeroRobot robot) throws Exception {

        is_sim_ = RobotBase.isSimulation() ;
    
        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Feeder motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////
        feeder_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Feeder.kMotorId,
                    IntakeShooterConstants.Feeder.kInvert,
                    IntakeShooterConstants.Feeder.kCurrentLimit);
        feeder_current_signal_ = feeder_motor_.getSupplyCurrent() ;                    

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // UpDown motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////                    
        updown_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.UpDown.kMotorId,
                    IntakeShooterConstants.UpDown.kInvert,
                    IntakeShooterConstants.UpDown.kCurrentLimit);
        updown_motor_.getPosition().setUpdateFrequency(100) ;
        updown_motor_.getVelocity().setUpdateFrequency(100) ;
        final Slot0Configs updownslot0cfg = new Slot0Configs()
                                .withKP(IntakeShooterConstants.UpDown.PID.kP)
                                .withKI(IntakeShooterConstants.UpDown.PID.kI)
                                .withKD(IntakeShooterConstants.UpDown.PID.kD)
                                .withKV(IntakeShooterConstants.UpDown.PID.kV)
                                .withKA(IntakeShooterConstants.UpDown.PID.kA)
                                .withKG(IntakeShooterConstants.UpDown.PID.kG)
                                .withKS(IntakeShooterConstants.UpDown.PID.kS) ;
        checkError("updown-set-PID-values", () -> updown_motor_.getConfigurator().apply(updownslot0cfg)) ;

        final MotionMagicConfigs updownmagiccfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(IntakeShooterConstants.UpDown.MotionMagic.kV)
                                .withMotionMagicAcceleration(IntakeShooterConstants.UpDown.MotionMagic.kA)
                                .withMotionMagicJerk(IntakeShooterConstants.UpDown.MotionMagic.kJ) ;
        checkError("updown-set-magic-motion", () -> updown_motor_.getConfigurator().apply(updownmagiccfg)) ;

        final SoftwareLimitSwitchConfigs updownlimitcfg = new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitEnable(true)
                            .withForwardSoftLimitThreshold(IntakeShooterConstants.UpDown.kMaxPosition / IntakeShooterConstants.UpDown.kDegreesPerRev)
                            .withReverseSoftLimitEnable(true)
                            .withReverseSoftLimitThreshold(IntakeShooterConstants.UpDown.kMinPosition / IntakeShooterConstants.UpDown.kDegreesPerRev) ;
        checkError("updown-set-software-limits", () -> updown_motor_.getConfigurator().apply(updownlimitcfg)) ;
        updown_position_signal_ = updown_motor_.getPosition() ;
        updown_velocity_signal_ = updown_motor_.getVelocity() ;
        updown_current_signal_ = updown_motor_.getSupplyCurrent() ;
        updown_voltage_signal_ = updown_motor_.getMotorVoltage() ;

        BaseStatusSignal.setUpdateFrequencyForAll(200,
            updown_position_signal_,
            updown_velocity_signal_,
            updown_current_signal_,
            updown_voltage_signal_) ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Tilt motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////          
        tilt_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Tilt.kMotorId,
                    IntakeShooterConstants.Tilt.kInvert,
                    IntakeShooterConstants.Tilt.kCurrentLimit);    

        final Slot0Configs tiltslot0cfg = new Slot0Configs()
                                .withKP(IntakeShooterConstants.Tilt.MovementPIDSlot0.kP)
                                .withKI(IntakeShooterConstants.Tilt.MovementPIDSlot0.kI)
                                .withKD(IntakeShooterConstants.Tilt.MovementPIDSlot0.kD)
                                .withKV(IntakeShooterConstants.Tilt.MovementPIDSlot0.kV)
                                .withKA(IntakeShooterConstants.Tilt.MovementPIDSlot0.kA)
                                .withKG(IntakeShooterConstants.Tilt.MovementPIDSlot0.kG)
                                .withKS(IntakeShooterConstants.Tilt.MovementPIDSlot0.kS) ;
        checkError("tilt-set-PID-value-movement", () -> tilt_motor_.getConfigurator().apply(tiltslot0cfg)) ;

        final Slot1Configs tiltslot1cfg = new Slot1Configs()
                                .withKP(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kP)
                                .withKI(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kI)
                                .withKD(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kD)
                                .withKV(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kV)
                                .withKA(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kA)
                                .withKG(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kG)
                                .withKS(IntakeShooterConstants.Tilt.TrackingPIDSlot1.kS) ;
        checkError("tilt-set-PID-value-tracking", () -> tilt_motor_.getConfigurator().apply(tiltslot1cfg)) ;        

        final MotionMagicConfigs tiltmagiccfg = new MotionMagicConfigs().withMotionMagicCruiseVelocity(IntakeShooterConstants.Tilt.MotionMagic.kV)
                                .withMotionMagicAcceleration(IntakeShooterConstants.Tilt.MotionMagic.kA)
                                .withMotionMagicJerk(IntakeShooterConstants.Tilt.MotionMagic.kJ) ;
        checkError("tilt-set-magic-motion", () -> tilt_motor_.getConfigurator().apply(tiltmagiccfg)) ;

        final SoftwareLimitSwitchConfigs tiltlimitcfg = new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitEnable(true)
                            .withForwardSoftLimitThreshold(IntakeShooterConstants.Tilt.kMaxPosition / IntakeShooterConstants.Tilt.kDegreesPerRev)
                            .withReverseSoftLimitEnable(true)
                            .withReverseSoftLimitThreshold(IntakeShooterConstants.Tilt.kMinPosition / IntakeShooterConstants.Tilt.kDegreesPerRev) ;
        checkError("tilt-set-software-limits", () -> tilt_motor_.getConfigurator().apply(tiltlimitcfg)) ;
        tilt_position_signal_ = tilt_motor_.getPosition() ;
        tilt_velocity_signal_ = tilt_motor_.getVelocity() ;
        tilt_current_signal_ = tilt_motor_.getSupplyCurrent() ;
        tilt_voltage_signal_ = tilt_motor_.getMotorVoltage() ;    
        
        BaseStatusSignal.setUpdateFrequencyForAll(200,
            tilt_position_signal_,
            tilt_velocity_signal_,
            tilt_current_signal_,
            tilt_voltage_signal_) ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Shooter1 motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////          
        shooter1_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter1.kMotorId,
                    IntakeShooterConstants.Shooter1.kInvert,
                    IntakeShooterConstants.Shooter1.kCurrentLimit);
        shooter1_motor_.getVelocity().setUpdateFrequency(100) ;
        shooter1_motor_.getPosition().setUpdateFrequency(100) ;        
        final Slot0Configs shooter1slot0cfg = new Slot0Configs().withKP(IntakeShooterConstants.Shooter.kP)
                                .withKI(IntakeShooterConstants.Shooter.kI)
                                .withKD(IntakeShooterConstants.Shooter.kD)
                                .withKV(IntakeShooterConstants.Shooter.kV)
                                .withKA(IntakeShooterConstants.Shooter.kA)
                                .withKG(IntakeShooterConstants.Shooter.kG)
                                .withKS(IntakeShooterConstants.Shooter.kS) ;
        checkError("set-shooter1-PID-value", () -> shooter1_motor_.getConfigurator().apply(shooter1slot0cfg)) ;
        shooter1_velocity_signal_ = shooter1_motor_.getVelocity() ;
        shooter1_current_signal_ = shooter1_motor_.getSupplyCurrent() ;
        shooter1_position_signal_ = shooter1_motor_.getPosition() ;
        shooter1_voltage_signal_ = shooter1_motor_.getMotorVoltage() ;        

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Shooter2 motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////         
        shooter2_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter2.kMotorId,
                    IntakeShooterConstants.Shooter2.kInvert,
                    IntakeShooterConstants.Shooter2.kCurrentLimit);
        shooter2_motor_.getVelocity().setUpdateFrequency(100) ;
        shooter2_motor_.getPosition().setUpdateFrequency(100) ;                   
        final Slot0Configs shooter2slot0cfg = new Slot0Configs().withKP(IntakeShooterConstants.Shooter.kP)
                                .withKI(IntakeShooterConstants.Shooter.kI)
                                .withKD(IntakeShooterConstants.Shooter.kD)
                                .withKV(IntakeShooterConstants.Shooter.kV)
                                .withKA(IntakeShooterConstants.Shooter.kA)
                                .withKG(IntakeShooterConstants.Shooter.kG)
                                .withKS(IntakeShooterConstants.Shooter.kS) ;
        checkError("set-shooter2-PID-value", () -> shooter2_motor_.getConfigurator().apply(shooter2slot0cfg)) ;
        shooter2_velocity_signal_ = shooter2_motor_.getVelocity() ;
        shooter2_current_signal_ = shooter2_motor_.getSupplyCurrent() ;  
        shooter2_position_signal_ = shooter2_motor_.getPosition() ;
        shooter2_voltage_signal_ = shooter2_motor_.getMotorVoltage() ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Note Sensor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////  
        note_sensor_ = new DigitalInput(IntakeShooterConstants.NoteSensor.kChannel) ;
        note_interrupt_ = new AsynchronousInterrupt(note_sensor_, (rising, falling) -> { noteInterruptHandler(rising, falling); }) ;
        note_interrupt_.setInterruptEdges(true, true);            
        note_interrupt_.enable();

        rising_seen_ = new AtomicBoolean() ;
        falling_seen_ = new AtomicBoolean() ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Absolute Encoder initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////          
        absolute_encoder_ = new AnalogInput(IntakeShooterConstants.Tilt.AbsoluteEncoder.kChannel) ;
        encoder_mapper_ = new EncoderMapper(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMin,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMin) ;

        encoder_mapper_.calibrate(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotCalibrationValue,
                                  IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderCalibrationValue(robot)) ;
        
        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Overall Phoenix 6 signal optimization
        /////////////////////////////////////////////////////////////////////////////////////////////////                                    
        checkError("set-intake-signals-update-frequeny", () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0,
                                            updown_position_signal_,
                                            updown_velocity_signal_,
                                            updown_current_signal_,
                                            updown_voltage_signal_,
                                            tilt_position_signal_,
                                            tilt_velocity_signal_,
                                            tilt_current_signal_,
                                            tilt_voltage_signal_,
                                            feeder_current_signal_,
                                            shooter1_velocity_signal_,
                                            shooter1_current_signal_,
                                            shooter1_position_signal_,
                                            shooter1_voltage_signal_,
                                            shooter2_velocity_signal_,
                                            shooter2_current_signal_,
                                            shooter2_position_signal_,
                                            shooter2_voltage_signal_)) ;

        checkError("updown-optimize-bus", () -> updown_motor_.optimizeBusUtilization()) ;
        checkError("tilt-optimize-bus", () -> tilt_motor_.optimizeBusUtilization()) ;
        checkError("shooter1-optimize-bus", () -> shooter1_motor_.optimizeBusUtilization()) ;
        checkError("shooter2-optimize-bus", () -> shooter2_motor_.optimizeBusUtilization()) ;
        checkError("feeder-optimize-bus", () -> feeder_motor_.optimizeBusUtilization()) ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Simulation initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////           
        if (RobotBase.isSimulation()) {
            updown_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), IntakeShooterConstants.UpDown.kSimGearRatio, IntakeShooterConstants.UpDown.kSimMotorLoad) ;
            tilt_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), IntakeShooterConstants.Tilt.kSimGearRatio, IntakeShooterConstants.Tilt.kSimMotorLoad) ;
            shooter1_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), IntakeShooterConstants.Shooter.kSimGearRatio, IntakeShooterConstants.Shooter.kSimMotorLoad);
            shooter2_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), IntakeShooterConstants.Shooter.kSimGearRatio, IntakeShooterConstants.Shooter.kSimMotorLoad) ;
            
            note_sim_ = new DIOSim(note_sensor_) ;
            note_sim_.setIsInput(true) ;
            note_sim_.setValue(true);
            note_sim_.setInitialized(true);
        }        
    }    

    public void updateInputs(IntakeShooterIOInputs inputs) {
        inputs.updownEncoder = updown_position_signal_.refresh().getValueAsDouble() ;
        inputs.updownPosition = updown_position_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
        inputs.updownVelocity = updown_velocity_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
        inputs.updownCurrent = updown_current_signal_.refresh().getValueAsDouble() ;
        inputs.updownVoltage = updown_voltage_signal_.refresh().getValueAsDouble() ;

        double tenc = tilt_position_signal_.refresh().getValueAsDouble() ;
        inputs.tiltPosition = tenc * IntakeShooterConstants.Tilt.kDegreesPerRev ;
        inputs.tiltVelocity = tilt_velocity_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
        inputs.tiltCurrent = tilt_current_signal_.refresh().getValueAsDouble() ;
        inputs.tiltVoltage = tilt_voltage_signal_.refresh().getValueAsDouble() ;
        inputs.tiltEncoder = tenc ;

        inputs.tiltAbsoluteEncoderPosition = getTiltAbsoluteEncoderPosition() ;

        inputs.feederCurrent = feeder_current_signal_.refresh().getValueAsDouble() ;

        inputs.shooter1Velocity = shooter1_velocity_signal_.refresh().getValueAsDouble() ;
        inputs.shooter1Current = shooter1_current_signal_.refresh().getValueAsDouble() ;
        inputs.shooter1Position = shooter1_position_signal_.refresh().getValueAsDouble() ;
        inputs.shooter1Voltage = shooter1_voltage_signal_.refresh().getValueAsDouble() ;

        inputs.shooter2Velocity = shooter2_velocity_signal_.refresh().getValueAsDouble() ;
        inputs.shooter2Current = shooter2_current_signal_.refresh().getValueAsDouble() ;
        inputs.shooter2Position = shooter2_position_signal_.refresh().getValueAsDouble() ;
        inputs.shooter2Voltage = shooter2_voltage_signal_.refresh().getValueAsDouble() ;

        inputs.risingEdge = rising_seen_.get() ;
        inputs.fallingEdge = falling_seen_.get() ;

        inputs.noteSensor = note_sensor_.get() ;

        rising_seen_.set(false) ;
        falling_seen_.set(false) ;
    }

    public void setUpDownTargetPos(double t) {
        updown_motor_.setControl(new MotionMagicVoltage(t / IntakeShooterConstants.UpDown.kDegreesPerRev)
                                    .withSlot(0)
                                    .withEnableFOC(true)) ;
    }

    public void setUpDownMotorPosition(double pos) {
        updown_motor_.setPosition(pos / IntakeShooterConstants.UpDown.kDegreesPerRev) ;
    }

    public void setUpDownMotorVoltage(double vol) {
        updown_voltage_ = vol ;
        updown_motor_.setControl(new VoltageOut(vol)) ;
    }    

    public void logUpDownMotor(SysIdRoutineLog log) {
        log.motor("updown")
            .voltage(Units.Volts.of(updown_voltage_))
            .angularPosition(Units.Revolutions.of(updown_position_signal_.refresh().getValueAsDouble()))
            .angularVelocity(Units.RevolutionsPerSecond.of(updown_velocity_signal_.refresh().getValueAsDouble())) ;
    }

    public double getTiltAbsoluteEncoderPosition() {
        return encoder_mapper_.toRobot(absolute_encoder_.getVoltage()) ;
    }

    public void setTiltTargetPos(boolean tracking, double t) {
        if (tracking) {
            tilt_motor_.setControl(new PositionVoltage(t / IntakeShooterConstants.Tilt.kDegreesPerRev)
                            .withSlot(1)
                            .withEnableFOC(true)) ;
        }
        else {
            tilt_motor_.setControl(new MotionMagicVoltage(t / IntakeShooterConstants.Tilt.kDegreesPerRev)
                            .withSlot(0)
                            .withEnableFOC(true)) ;
        }
    }    

    public void setTiltMotorPosition(double pos) {
        tilt_motor_.setPosition(pos / IntakeShooterConstants.Tilt.kDegreesPerRev) ;
    }

    public void setTiltMotorVoltage(double vol) {
        tilt_voltage_ = vol ;
        tilt_motor_.setControl(new VoltageOut(vol)) ;
    }

    public void logTiltMotor(SysIdRoutineLog log) {
        log.motor("tilt")
            .voltage(Units.Volts.of(tilt_voltage_))
            .angularPosition(Units.Revolutions.of(tilt_position_signal_.refresh().getValueAsDouble()))
            .angularVelocity(Units.RevolutionsPerSecond.of(tilt_velocity_signal_.refresh().getValueAsDouble()));
    }
 
    public void setShooter1Velocity(double vel) {
        shooter1_motor_.setControl(new VelocityVoltage(vel)) ;
    }

    public void setShooter1MotorVoltage(double vol) {
        shooter1_voltage_ = vol ;
        shooter1_motor_.setControl(new VoltageOut(vol)) ;
    }

    public void logShooter1Motor(SysIdRoutineLog log) {
        log.motor("shooter1")
            .voltage(Units.Volts.of(shooter1_voltage_))
            .angularPosition(Units.Revolutions.of(shooter1_position_signal_.refresh().getValueAsDouble()))
            .angularVelocity(Units.RevolutionsPerSecond.of(shooter1_velocity_signal_.refresh().getValueAsDouble()));
    }    

    public void setShooter2Velocity(double vel) {
        shooter2_motor_.setControl(new VelocityVoltage(vel)) ;
    }

    public void setShooter2MotorVoltage(double vol) {
        shooter2_voltage_ = vol ;
        shooter2_motor_.setControl(new VoltageOut(vol)) ;
    }    

    public void logShooter2Motor(SysIdRoutineLog log) {
        log.motor("shooter2")
            .voltage(Units.Volts.of(shooter2_voltage_))
            .angularPosition(Units.Revolutions.of(shooter2_position_signal_.refresh().getValueAsDouble()))
            .angularVelocity(Units.RevolutionsPerSecond.of(shooter2_velocity_signal_.refresh().getValueAsDouble())) ;
    }      

    public void setFeederMotorVoltage(double v) {
        feeder_voltage_ = v ;
        feeder_motor_.setControl(new VoltageOut(v)) ;
    }

    public double getFeederMotorVoltage() {
        return feeder_voltage_ ;
    }

    public void doSim(String name, TalonFX motor, DCMotorSim sim, double period){
        TalonFXSimState state = motor.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;
        sim.setInputVoltage(state.getMotorVoltage()) ;
        sim.update(period) ;
        state.setRawRotorPosition(sim.getAngularPositionRotations()) ;
        state.setRotorVelocity(edu.wpi.first.math.util.Units.radiansToRotations(sim.getAngularVelocityRadPerSec())) ;
    }

    public void simulate(double period) {
        doSim("tilt", tilt_motor_, tilt_sim_, period) ;
        doSim("updown", updown_motor_, updown_sim_, period) ;        
        doSim("shooter1", shooter1_motor_, shooter1_sim_, period) ;
        doSim("shooter2", shooter2_motor_, shooter2_sim_, period) ;
    }
    
    private void noteInterruptHandler(boolean rising, boolean falling) {
        if (rising) {
            if (is_sim_) {
                falling_seen_.set(true) ;
            }
            else {
                rising_seen_.set(true) ;                
            }
        }

        if (falling) {
            if (is_sim_) {
                rising_seen_.set(true) ;
            }
            else {
                falling_seen_.set(true) ;
            }
        }
    }

    private static void checkError(String msg, Supplier<StatusCode> toApply) throws Exception {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = kApplyTries ;
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            msg = msg + " - " + code.toString() ;
            throw new Exception(msg) ;
        }
    }      

    public List<TalonFX> getCTREMotors() {
        return Arrays.asList(feeder_motor_, updown_motor_, shooter1_motor_, shooter2_motor_, tilt_motor_ ) ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }     
}
