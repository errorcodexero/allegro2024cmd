package frc.robot.subsystems.intakeshooter;

import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import org.xero1425.base.TalonFXFactory;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.EncoderMapper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.Units ;

public class IntakeShooterIOHardware implements IntakeShooterIO {

    private final static int kApplyTries = 5 ;  

    private HashMap<String, TalonFX> talon_motors_ ;

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
    private AtomicInteger shooter_position_ ;
    private MedianFilter abs_encoder_median_filter_filter_ ;

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


        abs_encoder_median_filter_filter_ = new MedianFilter(3) ;

        talon_motors_ = new HashMap<>() ;

        //
        // Initialize all of the hardware
        //
        initFeederMotor() ;              
        initUpDownMotor() ;
        initTiltMotor();
        initShooterMotors() ;
        initNoteSensor() ;
        initAbsoluteEncoder(robot) ;
       
        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Overall Phoenix 6 signal optimization
        /////////////////////////////////////////////////////////////////////////////////////////////////                                    
        checkError("set-intake-signals-update-frequeny-fast", () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0,
                                            updown_position_signal_,
                                            updown_velocity_signal_,
                                            updown_current_signal_,
                                            updown_voltage_signal_,
                                            tilt_position_signal_,
                                            tilt_velocity_signal_,
                                            tilt_current_signal_,
                                            tilt_voltage_signal_,
                                            shooter1_velocity_signal_,
                                            shooter1_current_signal_,
                                            shooter1_position_signal_,
                                            shooter1_voltage_signal_,
                                            shooter2_velocity_signal_,
                                            shooter2_current_signal_,
                                            shooter2_position_signal_,
                                            shooter2_voltage_signal_)) ;

        checkError("set-intake-signals-update-frequeny-slow", () -> BaseStatusSignal.setUpdateFrequencyForAll(1.0,
                                            feeder_current_signal_)) ;                                            

        checkError("updown-optimize-bus", () -> updown_motor_.optimizeBusUtilization()) ;
        checkError("tilt-optimize-bus", () -> tilt_motor_.optimizeBusUtilization()) ;
        checkError("shooter1-optimize-bus", () -> shooter1_motor_.optimizeBusUtilization()) ;
        checkError("shooter2-optimize-bus", () -> shooter2_motor_.optimizeBusUtilization()) ;
        checkError("feeder-optimize-bus", () -> feeder_motor_.optimizeBusUtilization()) ;      
    }

    public double getShooterPositionAtRisingEdge() {
        return shooter_position_.get() / 10000.0 ;
    }

    public void updateInputs(IntakeShooterIOInputs inputs) {
        inputs.updownEncoder = updown_position_signal_.refresh().getValueAsDouble() ;
        inputs.updownPosition = updown_position_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
        inputs.updownVelocity = updown_velocity_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
        inputs.updownCurrent = updown_current_signal_.refresh().getValueAsDouble() ;
        inputs.updownVoltage = updown_voltage_signal_.refresh().getValueAsDouble() ;

        inputs.tiltEncoder = tilt_position_signal_.refresh().getValueAsDouble() ;
        inputs.tiltPosition = tilt_position_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
        inputs.tiltVelocity = tilt_velocity_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
        inputs.tiltCurrent = tilt_current_signal_.refresh().getValueAsDouble() ;
        inputs.tiltVoltage = tilt_voltage_signal_.refresh().getValueAsDouble() ;

        inputs.tiltAbsoluteEncoderPosition = getTiltAbsoluteEncoderPosition() ;
        inputs.tiltAbsoluteEncoderPositionMedian = abs_encoder_median_filter_filter_.calculate(inputs.tiltAbsoluteEncoderPosition) ;

        inputs.feederCurrent = feeder_current_signal_.refresh().getValueAsDouble() ;

        inputs.shooter1Velocity = shooter1_velocity_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.Shooter.kShooterRevsPerMotorRev ;
        inputs.shooter1Current = shooter1_current_signal_.refresh().getValueAsDouble()  * IntakeShooterConstants.Shooter.kShooterRevsPerMotorRev ;
        inputs.shooter1Position = shooter1_position_signal_.refresh().getValueAsDouble() ;
        inputs.shooter1Voltage = shooter1_voltage_signal_.refresh().getValueAsDouble() ;

        inputs.shooter2Velocity = shooter2_velocity_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.Shooter.kShooterRevsPerMotorRev ;
        inputs.shooter2Current = shooter2_current_signal_.refresh().getValueAsDouble() * IntakeShooterConstants.Shooter.kShooterRevsPerMotorRev ;
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
        tracking = false ;
        if (tracking) {
            tilt_motor_.setControl(new PositionVoltage(t / IntakeShooterConstants.Tilt.kDegreesPerRev)
                            .withSlot(1)
                            .withEnableFOC(true)) ;
        }
        else {
            double target = t / IntakeShooterConstants.Tilt.kDegreesPerRev ;
            tilt_motor_.setControl(new MotionMagicVoltage(target)
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
        double cvel = vel / IntakeShooterConstants.Shooter.kShooterRevsPerMotorRev ;
        shooter1_motor_.setControl(new MotionMagicVelocityVoltage(cvel)) ;
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
        double cvel = vel / IntakeShooterConstants.Shooter.kShooterRevsPerMotorRev ;
        shooter2_motor_.setControl(new MotionMagicVelocityVoltage(cvel)) ;
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

    private void noteInterruptHandler(boolean rising, boolean falling) {
        if (rising) {
            if (RobotBase.isReal())
                rising_seen_.set(true) ;
            else
                falling_seen_.set(true) ;

            shooter_position_.set((int)(shooter1_position_signal_.refresh().getValueAsDouble() * 10000)) ;
        }

        if (falling) {
            if (RobotBase.isReal())
                falling_seen_.set(true) ;
            else
                rising_seen_.set(true) ;
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

    public Map<String, TalonFX> getCTREMotors() {
        return talon_motors_ ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }

    private void initFeederMotor() throws Exception {
        feeder_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Feeder.kMotorId,
                    IntakeShooterConstants.Feeder.kInvert,
                    IntakeShooterConstants.Feeder.kCurrentLimit);
        talon_motors_.put(IntakeShooterSubsystem.FEEDER_MOTOR_NAME, feeder_motor_) ; 
        feeder_current_signal_ = feeder_motor_.getSupplyCurrent() ;      
    }

    private void initUpDownMotor() throws Exception {
        updown_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.UpDown.kMotorId,
                    IntakeShooterConstants.UpDown.kInvert,
                    IntakeShooterConstants.UpDown.kCurrentLimit);
        talon_motors_.put(IntakeShooterSubsystem.UPDOWN_MOTOR_NAME, updown_motor_) ;                     

        if (XeroRobot.isReal()) {
            final Slot0Configs updownslot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.UpDown.Real.PID.kP)
                                    .withKI(IntakeShooterConstants.UpDown.Real.PID.kI)
                                    .withKD(IntakeShooterConstants.UpDown.Real.PID.kD)
                                    .withKV(IntakeShooterConstants.UpDown.Real.PID.kV)
                                    .withKA(IntakeShooterConstants.UpDown.Real.PID.kA)
                                    .withKG(IntakeShooterConstants.UpDown.Real.PID.kG)
                                    .withKS(IntakeShooterConstants.UpDown.Real.PID.kS) ;
            checkError("updown-set-PID-values", () -> updown_motor_.getConfigurator().apply(updownslot0cfg)) ;

            final MotionMagicConfigs updownmagiccfg = new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(IntakeShooterConstants.UpDown.Real.MotionMagic.kV)
                                    .withMotionMagicAcceleration(IntakeShooterConstants.UpDown.Real.MotionMagic.kA)
                                    .withMotionMagicJerk(IntakeShooterConstants.UpDown.Real.MotionMagic.kJ) ;
            checkError("updown-set-magic-motion", () -> updown_motor_.getConfigurator().apply(updownmagiccfg)) ;
        }
        else 
        {
            final Slot0Configs updownslot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.UpDown.Simulated.PID.kP)
                                    .withKI(IntakeShooterConstants.UpDown.Simulated.PID.kI)
                                    .withKD(IntakeShooterConstants.UpDown.Simulated.PID.kD)
                                    .withKV(IntakeShooterConstants.UpDown.Simulated.PID.kV)
                                    .withKA(IntakeShooterConstants.UpDown.Simulated.PID.kA)
                                    .withKG(IntakeShooterConstants.UpDown.Simulated.PID.kG)
                                    .withKS(IntakeShooterConstants.UpDown.Simulated.PID.kS) ;
            checkError("updown-set-PID-values", () -> updown_motor_.getConfigurator().apply(updownslot0cfg)) ;

            final MotionMagicConfigs updownmagiccfg = new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(IntakeShooterConstants.UpDown.Simulated.MotionMagic.kV)
                                    .withMotionMagicAcceleration(IntakeShooterConstants.UpDown.Simulated.MotionMagic.kA)
                                    .withMotionMagicJerk(IntakeShooterConstants.UpDown.Simulated.MotionMagic.kJ) ;
            checkError("updown-set-magic-motion", () -> updown_motor_.getConfigurator().apply(updownmagiccfg)) ;          
        }

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
    }

    private void initTiltMotor() throws Exception {
        tilt_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Tilt.kMotorId,
                    IntakeShooterConstants.Tilt.kInvert,
                    IntakeShooterConstants.Tilt.kCurrentLimit);    
        talon_motors_.put(IntakeShooterSubsystem.TILT_MOTOR_NAME, tilt_motor_) ; 

        if (XeroRobot.isReal()) 
        {
            final Slot0Configs tiltslot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kP)
                                    .withKI(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kI)
                                    .withKD(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kD)
                                    .withKV(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kV)
                                    .withKA(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kA)
                                    .withKG(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kG)
                                    .withKS(IntakeShooterConstants.Tilt.Real.MovementPIDSlot0.kS) ;
            checkError("tilt-set-PID-value-movement", () -> tilt_motor_.getConfigurator().apply(tiltslot0cfg)) ;

            final MotionMagicConfigs tiltmagiccfg = new MotionMagicConfigs().withMotionMagicCruiseVelocity(IntakeShooterConstants.Tilt.Real.MotionMagic.kV)
                                    .withMotionMagicAcceleration(IntakeShooterConstants.Tilt.Real.MotionMagic.kA)
                                    .withMotionMagicJerk(IntakeShooterConstants.Tilt.Real.MotionMagic.kJ) ;
            checkError("tilt-set-magic-motion", () -> tilt_motor_.getConfigurator().apply(tiltmagiccfg)) ;
        }
        else
        {
            final Slot0Configs tiltslot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kP)
                                    .withKI(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kI)
                                    .withKD(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kD)
                                    .withKV(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kV)
                                    .withKA(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kA)
                                    .withKG(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kG)
                                    .withKS(IntakeShooterConstants.Tilt.Simulated.MovementPIDSlot0.kS) ;
            checkError("tilt-set-PID-value-movement", () -> tilt_motor_.getConfigurator().apply(tiltslot0cfg)) ;

            final MotionMagicConfigs tiltmagiccfg = new MotionMagicConfigs().withMotionMagicCruiseVelocity(IntakeShooterConstants.Tilt.Simulated.MotionMagic.kV)
                                    .withMotionMagicAcceleration(IntakeShooterConstants.Tilt.Simulated.MotionMagic.kA)
                                    .withMotionMagicJerk(IntakeShooterConstants.Tilt.Simulated.MotionMagic.kJ) ;
            checkError("tilt-set-magic-motion", () -> tilt_motor_.getConfigurator().apply(tiltmagiccfg)) ;         
        }

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
    }

    void initShooterMotors() throws Exception {
        shooter1_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter1.kMotorId,
                    IntakeShooterConstants.Shooter1.kInvert,
                    IntakeShooterConstants.Shooter1.kCurrentLimit);
        talon_motors_.put(IntakeShooterSubsystem.SHOOTER1_MOTOR_NAME, shooter1_motor_) ;                    

        if (XeroRobot.isReal())
        {
            final Slot0Configs shooter1slot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.Shooter1.Real.Pid.kP)
                                    .withKI(IntakeShooterConstants.Shooter1.Real.Pid.kI)
                                    .withKD(IntakeShooterConstants.Shooter1.Real.Pid.kD)
                                    .withKV(IntakeShooterConstants.Shooter1.Real.Pid.kV)
                                    .withKA(IntakeShooterConstants.Shooter1.Real.Pid.kA)
                                    .withKG(IntakeShooterConstants.Shooter1.Real.Pid.kG)
                                    .withKS(IntakeShooterConstants.Shooter1.Real.Pid.kS) ;
            checkError("set-shooter1-PID-value", () -> shooter1_motor_.getConfigurator().apply(shooter1slot0cfg)) ;
        }
        else
        {
            final Slot0Configs shooter1slot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.Shooter.Simulated.kP)
                                    .withKI(IntakeShooterConstants.Shooter.Simulated.kI)
                                    .withKD(IntakeShooterConstants.Shooter.Simulated.kD)
                                    .withKV(IntakeShooterConstants.Shooter.Simulated.kV)
                                    .withKA(IntakeShooterConstants.Shooter.Simulated.kA)
                                    .withKG(IntakeShooterConstants.Shooter.Simulated.kG)
                                    .withKS(IntakeShooterConstants.Shooter.Simulated.kS) ;
            checkError("set-shooter1-PID-value", () -> shooter1_motor_.getConfigurator().apply(shooter1slot0cfg)) ;            
        }

        final MotionMagicConfigs shooter1cfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(IntakeShooterConstants.Shooter1.Real.MotionMagic.kV)
                                .withMotionMagicAcceleration(IntakeShooterConstants.Shooter1.Real.MotionMagic.kA)
                                .withMotionMagicJerk(IntakeShooterConstants.Shooter1.Real.MotionMagic.kJ) ;
        checkError("updown-set-magic-motion", () -> shooter1_motor_.getConfigurator().apply(shooter1cfg)) ;

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
        talon_motors_.put(IntakeShooterSubsystem.SHOOTER2_MOTOR_NAME, shooter2_motor_) ;                     
                  
        if (XeroRobot.isReal())
        {
            final Slot0Configs shooter2slot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.Shooter2.Real.Pid.kP)
                                    .withKI(IntakeShooterConstants.Shooter2.Real.Pid.kI)
                                    .withKD(IntakeShooterConstants.Shooter2.Real.Pid.kD)
                                    .withKV(IntakeShooterConstants.Shooter2.Real.Pid.kV)
                                    .withKA(IntakeShooterConstants.Shooter2.Real.Pid.kA)
                                    .withKG(IntakeShooterConstants.Shooter2.Real.Pid.kG)
                                    .withKS(IntakeShooterConstants.Shooter2.Real.Pid.kS) ;
            checkError("set-shooter1-PID-value", () -> shooter2_motor_.getConfigurator().apply(shooter2slot0cfg)) ;
        }
        else
        {
            final Slot0Configs shooter2slot0cfg = new Slot0Configs()
                                    .withKP(IntakeShooterConstants.Shooter.Simulated.kP)
                                    .withKI(IntakeShooterConstants.Shooter.Simulated.kI)
                                    .withKD(IntakeShooterConstants.Shooter.Simulated.kD)
                                    .withKV(IntakeShooterConstants.Shooter.Simulated.kV)
                                    .withKA(IntakeShooterConstants.Shooter.Simulated.kA)
                                    .withKG(IntakeShooterConstants.Shooter.Simulated.kG)
                                    .withKS(IntakeShooterConstants.Shooter.Simulated.kS) ;
            checkError("set-shooter1-PID-value", () -> shooter2_motor_.getConfigurator().apply(shooter2slot0cfg)) ;            
        }

        final MotionMagicConfigs shooter2cfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(IntakeShooterConstants.Shooter1.Real.MotionMagic.kV)
                                .withMotionMagicAcceleration(IntakeShooterConstants.Shooter1.Real.MotionMagic.kA)
                                .withMotionMagicJerk(IntakeShooterConstants.Shooter1.Real.MotionMagic.kJ) ;
        checkError("updown-set-magic-motion", () -> shooter2_motor_.getConfigurator().apply(shooter2cfg)) ;        

        shooter2_velocity_signal_ = shooter2_motor_.getVelocity() ;
        shooter2_current_signal_ = shooter2_motor_.getSupplyCurrent() ;  
        shooter2_position_signal_ = shooter2_motor_.getPosition() ;
        shooter2_voltage_signal_ = shooter2_motor_.getMotorVoltage() ;
    }

    private void initNoteSensor() {
        note_sensor_ = new DigitalInput(IntakeShooterConstants.NoteSensor.kChannel) ;
        note_interrupt_ = new AsynchronousInterrupt(note_sensor_, (rising, falling) -> { noteInterruptHandler(rising, falling); }) ;
        note_interrupt_.setInterruptEdges(true, true);            
        note_interrupt_.enable();

        rising_seen_ = new AtomicBoolean() ;
        falling_seen_ = new AtomicBoolean() ;
        shooter_position_ = new AtomicInteger() ;
    }

    private void initAbsoluteEncoder(XeroRobot robot) {
        absolute_encoder_ = new AnalogInput(IntakeShooterConstants.Tilt.AbsoluteEncoder.kChannel) ;
        encoder_mapper_ = new EncoderMapper(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMin,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMin) ;

        encoder_mapper_.calibrate(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotCalibrationValue,
                                  IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderCalibrationValue(robot)) ;
    }    
}
