package frc.robot.subsystems.tramp;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.xero1425.base.TalonFXFactory;
import org.xero1425.base.XeroRobot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Units ;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class TrampIOHardware implements TrampIO {
    private final static int kApplyTries = 5 ;

    private HashMap<String, TalonFX> motors_ ;

    private TalonFX elevator_motor_ ;
    private TalonFX arm_motor_ ;
    private TalonFX climber_motor_ ;
    private CANSparkFlex manipulator_motor_ ;
    private RelativeEncoder manipulator_encoder_ ;
    private SparkPIDController manipulator_pid_ ;

    private StatusSignal<Double> elevator_pos_sig_ ;
    private StatusSignal<Double> elevator_vel_sig_ ;
    private StatusSignal<Double> elevator_current_sig_ ;
    private StatusSignal<Double> elevator_output_sig_ ;

    private StatusSignal<Double> arm_pos_sig_ ;
    private StatusSignal<Double> arm_vel_sig_ ;
    private StatusSignal<Double> arm_current_sig_ ;
    private StatusSignal<Double> arm_output_sig_ ;

    private StatusSignal<Double> climber_current_sig_ ;
    private StatusSignal<Double> climber_pos_sig_ ;
    private StatusSignal<Double> climber_output_sig_ ;
    private StatusSignal<Double> climber_velocity_sig_ ;

    private double manipulator_voltage_ ;
    private double climber_voltage_ ;
    private double arm_voltage_ ;
    private double elevator_voltage_ ;

    private Encoder thru_bore_encoder_ ;

    public TrampIOHardware() throws Exception {

        motors_ = new HashMap<>() ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Elevator motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////
        elevator_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Elevator.kMotorId,
                                                                    TrampConstants.Elevator.kInverted,
                                                                    TrampConstants.Elevator.kCurrentLimit);
                                                                    elevator_motor_.setPosition(0.0) ;
        motors_.put(TrampSubsystem.ELEVATOR_MOTOR_NAME, elevator_motor_) ;
        final Slot0Configs elevatorcfg ;

        if (XeroRobot.isReal()) {
            elevatorcfg = new Slot0Configs().withKP(TrampConstants.Elevator.Real.PID.kP)
                                    .withKI(TrampConstants.Elevator.Real.PID.kI)
                                    .withKD(TrampConstants.Elevator.Real.PID.kD)
                                    .withKV(TrampConstants.Elevator.Real.PID.kV)
                                    .withKA(TrampConstants.Elevator.Real.PID.kA)
                                    .withKG(TrampConstants.Elevator.Real.PID.kG)
                                    .withKS(TrampConstants.Elevator.Real.PID.kS)
                                    .withGravityType(GravityTypeValue.Elevator_Static) ;
            checkError("set-elevator-PID-value", () -> elevator_motor_.getConfigurator().apply(elevatorcfg)) ;

            final MotionMagicConfigs mmcfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(TrampConstants.Elevator.Real.MotionMagic.kMaxVelocity)
                                .withMotionMagicAcceleration(TrampConstants.Elevator.Real.MotionMagic.kMaxAcceleration)
                                .withMotionMagicJerk(TrampConstants.Elevator.Real.MotionMagic.kJerk) ;
            checkError("set-elevator-motion-magic", () -> elevator_motor_.getConfigurator().apply(mmcfg)) ;

        }
        else {
            elevatorcfg = new Slot0Configs().withKP(TrampConstants.Elevator.Simulated.PID.kP)
                                    .withKI(TrampConstants.Elevator.Simulated.PID.kI)
                                    .withKD(TrampConstants.Elevator.Simulated.PID.kD)
                                    .withKV(TrampConstants.Elevator.Simulated.PID.kV)
                                    .withKA(TrampConstants.Elevator.Simulated.PID.kA)
                                    .withKG(TrampConstants.Elevator.Simulated.PID.kG)
                                    .withKS(TrampConstants.Elevator.Simulated.PID.kS)
                                    .withGravityType(GravityTypeValue.Elevator_Static) ;
            checkError("set-elevator-PID-value", () -> elevator_motor_.getConfigurator().apply(elevatorcfg)) ;

            final MotionMagicConfigs mmcfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(TrampConstants.Elevator.Simulated.MotionMagic.kMaxVelocity)
                                .withMotionMagicAcceleration(TrampConstants.Elevator.Simulated.MotionMagic.kMaxAcceleration)
                                .withMotionMagicJerk(TrampConstants.Elevator.Simulated.MotionMagic.kJerk) ;
            checkError("set-elevator-motion-magic", () -> elevator_motor_.getConfigurator().apply(mmcfg)) ;
        }

        final SoftwareLimitSwitchConfigs elevlimitcfg = new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitEnable(true)
                            .withForwardSoftLimitThreshold(TrampConstants.Elevator.kMaxPosition / TrampConstants.Elevator.kMetersPerRev)
                            .withReverseSoftLimitEnable(true)
                            .withReverseSoftLimitThreshold(TrampConstants.Elevator.kMinPosition / TrampConstants.Elevator.kMetersPerRev) ;
        checkError("set-elevator-soft-limit-value", () -> elevator_motor_.getConfigurator().apply(elevlimitcfg)) ;

        elevator_pos_sig_ = elevator_motor_.getPosition() ;
        elevator_vel_sig_ = elevator_motor_.getVelocity() ;
        elevator_current_sig_ = elevator_motor_.getSupplyCurrent() ;
        elevator_output_sig_ = elevator_motor_.getClosedLoopOutput() ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Arm motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////
        arm_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Arm.kMotorId,
                                                               TrampConstants.Arm.kInverted,
                                                               TrampConstants.Arm.kCurrentLimit);
        motors_.put(TrampSubsystem.ARM_MOTOR_NAME, arm_motor_) ;

        if (XeroRobot.isReal()) {
            final Slot0Configs armcfg = new Slot0Configs().withKP(TrampConstants.Arm.Real.PID.kP)
                                    .withKI(TrampConstants.Arm.Real.PID.kI)
                                    .withKD(TrampConstants.Arm.Real.PID.kD)
                                    .withKV(TrampConstants.Arm.Real.PID.kV)
                                    .withKA(TrampConstants.Arm.Real.PID.kA)
                                    .withKG(TrampConstants.Arm.Real.PID.kG)
                                    .withKS(TrampConstants.Arm.Real.PID.kS)
                                    .withGravityType(GravityTypeValue.Arm_Cosine) ;
            checkError("set-arm-PID-value", () -> arm_motor_.getConfigurator().apply(armcfg)) ;

            final MotionMagicConfigs armmmcfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(TrampConstants.Arm.Real.MotionMagic.kMaxVelocity)
                                .withMotionMagicAcceleration(TrampConstants.Arm.Real.MotionMagic.kMaxAcceleration)
                                .withMotionMagicJerk(TrampConstants.Arm.Real.MotionMagic.kJerk) ;
            checkError("set-elevator-motion-magic", () -> arm_motor_.getConfigurator().apply(armmmcfg)) ;
        }
        else {
            final Slot0Configs armcfg = new Slot0Configs().withKP(TrampConstants.Arm.Simulated.PID.kP)
                                    .withKI(TrampConstants.Arm.Simulated.PID.kI)
                                    .withKD(TrampConstants.Arm.Simulated.PID.kD)
                                    .withKV(TrampConstants.Arm.Simulated.PID.kV)
                                    .withKA(TrampConstants.Arm.Simulated.PID.kA)
                                    .withKG(TrampConstants.Arm.Simulated.PID.kG)
                                    .withKS(TrampConstants.Arm.Simulated.PID.kS)
                                    .withGravityType(GravityTypeValue.Arm_Cosine) ;
            checkError("set-arm-PID-value", () -> arm_motor_.getConfigurator().apply(armcfg)) ;

            final MotionMagicConfigs armmmcfg = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(TrampConstants.Arm.Simulated.MotionMagic.kMaxVelocity)
                                .withMotionMagicAcceleration(TrampConstants.Arm.Simulated.MotionMagic.kMaxAcceleration)
                                .withMotionMagicJerk(TrampConstants.Arm.Simulated.MotionMagic.kJerk) ;
            checkError("set-elevator-motion-magic", () -> arm_motor_.getConfigurator().apply(armmmcfg)) ;
        }

        final SoftwareLimitSwitchConfigs armlimitcfg = new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitEnable(true)
                            .withForwardSoftLimitThreshold(TrampConstants.Arm.kMaxPosition)
                            .withReverseSoftLimitEnable(true)
                            .withReverseSoftLimitThreshold(TrampConstants.Arm.kMinPosition) ;
        checkError("set-elevator-soft-limit-value", () -> arm_motor_.getConfigurator().apply(armlimitcfg)) ;

        arm_pos_sig_ = arm_motor_.getPosition() ;
        arm_vel_sig_ = arm_motor_.getVelocity() ;
        arm_current_sig_ = arm_motor_.getSupplyCurrent() ;
        arm_output_sig_ = arm_motor_.getClosedLoopOutput() ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Climber motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////
        climber_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Climber.kMotorId,
                                                                   TrampConstants.Climber.kInverted,
                                                                   TrampConstants.Climber.kCurrentLimit);
        motors_.put(TrampSubsystem.CLIMBER_MOTOR_NAME, climber_motor_) ;
        climber_pos_sig_ = climber_motor_.getPosition() ;
        climber_current_sig_ = climber_motor_.getSupplyCurrent() ;
        climber_output_sig_ = climber_motor_.getClosedLoopOutput();
        climber_velocity_sig_ = climber_motor_.getVelocity() ;
        climber_voltage_ = 0.0 ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Manipulator motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////
        manipulator_motor_ = new CANSparkFlex(TrampConstants.Manipulator.kMotorId, CANSparkFlex.MotorType.kBrushless);
        manipulator_motor_.setInverted(TrampConstants.Manipulator.kInverted);
        manipulator_motor_.setSmartCurrentLimit(60) ;
        manipulator_motor_.setIdleMode(IdleMode.kBrake) ;
        manipulator_motor_.enableVoltageCompensation(11.0) ;
        manipulator_encoder_ = manipulator_motor_.getEncoder() ;
        manipulator_encoder_.setVelocityConversionFactor(1.0 / 60.0) ;  // Convert from RPM to RPS
        manipulator_voltage_ = 0.0 ;
        manipulator_pid_ = manipulator_motor_.getPIDController() ;

        manipulator_pid_.setP(TrampConstants.Manipulator.PositionPID.kP, 0) ;
        manipulator_pid_.setI(TrampConstants.Manipulator.PositionPID.kI, 0) ;
        manipulator_pid_.setD(TrampConstants.Manipulator.PositionPID.kD, 0) ;
        manipulator_pid_.setFF(TrampConstants.Manipulator.PositionPID.kV, 0) ;

        manipulator_pid_.setP(TrampConstants.Manipulator.VelocityPID.kP, 1) ;
        manipulator_pid_.setI(TrampConstants.Manipulator.VelocityPID.kI, 1) ;
        manipulator_pid_.setD(TrampConstants.Manipulator.VelocityPID.kD, 1) ;
        manipulator_pid_.setFF(TrampConstants.Manipulator.VelocityPID.kV, 1) ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Thru Bore Encoder
        /////////////////////////////////////////////////////////////////////////////////////////////////
        thru_bore_encoder_ = new Encoder(TrampConstants.Manipulator.ThruBoreEncoder.kEncoderA,
                                            TrampConstants.Manipulator.ThruBoreEncoder.kEncoderB,
                                            TrampConstants.Manipulator.ThruBoreEncoder.kEncoderInverted,
                                            Encoder.EncodingType.k1X) ;
        thru_bore_encoder_.setDistancePerPulse(TrampConstants.Manipulator.ThruBoreEncoder.kEncoderDistancePerPulse) ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Overall Phoenix 6 signal optimization
        /////////////////////////////////////////////////////////////////////////////////////////////////
        BaseStatusSignal.setUpdateFrequencyForAll(75.0,
                                elevator_pos_sig_,
                                elevator_vel_sig_,
                                elevator_current_sig_,
                                elevator_output_sig_,
                                arm_pos_sig_,
                                arm_vel_sig_,
                                arm_current_sig_,
                                arm_output_sig_,
                                climber_current_sig_,
                                climber_pos_sig_,
                                climber_output_sig_,
                                climber_velocity_sig_) ;

        checkError("elevator-bus-optimization", () -> elevator_motor_.optimizeBusUtilization()) ;
        checkError("arm-bus-optimization", () -> arm_motor_.optimizeBusUtilization()) ;
        checkError("climber-bus-optimization", () -> climber_motor_.optimizeBusUtilization()) ;
    }

    public void updateInputs(TrampIOInputs inputs) {
        double enc ;

        enc = elevator_pos_sig_.refresh().getValueAsDouble() ;
        inputs.elevatorPosition = enc * TrampConstants.Elevator.kMetersPerRev ;
        inputs.elevatorVelocity = elevator_vel_sig_.refresh().getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;
        inputs.elevatorCurrent = elevator_current_sig_.refresh().getValueAsDouble() ;
        inputs.elevatorOutput = elevator_output_sig_.refresh().getValueAsDouble() ;
        inputs.elevatorEncoder = enc ;

        enc = arm_pos_sig_.refresh().getValueAsDouble() ;
        inputs.armPosition = enc * TrampConstants.Arm.kDegreesPerRev ;
        inputs.armVelocity = arm_vel_sig_.refresh().getValueAsDouble() * TrampConstants.Arm.kDegreesPerRev ;
        inputs.armCurrent = arm_current_sig_.refresh().getValueAsDouble() ;
        inputs.armOutput = arm_output_sig_.refresh().getValueAsDouble() ;
        inputs.armEncoder = enc ;

        enc = climber_pos_sig_.refresh().getValueAsDouble() ;
        inputs.climberPosition = enc * TrampConstants.Climber.kMetersPerRev ;
        inputs.climberCurrent = climber_current_sig_.refresh().getValueAsDouble() ;
        inputs.climberOutput = climber_output_sig_.refresh().getValueAsDouble() ;
        inputs.climberVelocity = climber_velocity_sig_.refresh().getValueAsDouble() ;
        inputs.climberEncoder = enc ;

        inputs.manipulatorPosition = manipulator_encoder_.getPosition() ;
        inputs.manipulatorVelocity = manipulator_encoder_.getVelocity() ;
        inputs.manipulatorCurrent = manipulator_motor_.getOutputCurrent() ;

        inputs.manipulatorFreeWheelPosition = thru_bore_encoder_.getDistance() ;
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // Elevator
    //
    ////////////////////////////////////////////////////////////////////////////

    public void setElevatorTargetPos(double pos) {
        elevator_motor_.setControl(new MotionMagicVoltage(pos / TrampConstants.Elevator.kMetersPerRev)
                                    .withSlot(0)
                                    .withEnableFOC(true)) ;
    }

    public void setElevatorMotorVoltage(double volts) {
        elevator_voltage_ = volts ;
        elevator_motor_.setControl(new VoltageOut(elevator_voltage_)) ;
    }

    public double getElevatorMotorVoltage() {
        return elevator_voltage_ ;
    }

    public void logElevatorMotor(SysIdRoutineLog log) {
        double pos = elevator_pos_sig_.refresh().getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;
        double vel = elevator_vel_sig_.refresh().getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;

        log.motor("elevator")
            .voltage(Units.Volts.of(elevator_voltage_))
            .linearPosition(Units.Meters.of(pos))
            .linearVelocity(Units.MetersPerSecond.of(vel)) ;
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // Arm
    //
    ////////////////////////////////////////////////////////////////////////////

    public void setArmTargetPos(double pos) {
        arm_motor_.setControl(new MotionMagicVoltage(pos / TrampConstants.Arm.kDegreesPerRev)
                                    .withSlot(0)
                                    .withEnableFOC(true)) ;
    }

    public void setArmPosition(double pos) {
        arm_motor_.setPosition(pos / TrampConstants.Arm.kDegreesPerRev) ;
    }

    public void setArmMotorVoltage(double volts) {
        arm_voltage_ = volts ;
        arm_motor_.setControl(new VoltageOut(arm_voltage_));
    }

    public double getArmMotorVoltage() {
        return arm_voltage_ ;
    }

    public void logArmMotor(SysIdRoutineLog log) {
        log.motor("arm")
            .voltage(Units.Volts.of(arm_voltage_))
            .angularPosition(Units.Revolutions.of(arm_pos_sig_.refresh().getValueAsDouble()))
            .angularVelocity(Units.RevolutionsPerSecond.of(arm_vel_sig_.refresh().getValueAsDouble())) ;
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // Manipulator
    //
    ////////////////////////////////////////////////////////////////////////////

    public void setManipulatorTargetPosition(double pos) {
        manipulator_pid_.setReference(pos, CANSparkBase.ControlType.kPosition, 0) ;
        manipulator_voltage_ = 0.0 ;
    }

    public void setManipulatorTargetVelocity(double vel) {
        manipulator_pid_.setReference(vel, CANSparkBase.ControlType.kVelocity, 1) ;
        manipulator_voltage_ = 0.0 ;
    }

    public void setManipulatorVoltage(double volts) {
        manipulator_voltage_ = volts ;
        manipulator_motor_.setVoltage(volts);
    }

    public double getManipulatorVoltage() {
        return manipulator_voltage_ ;
    }

    public void logManipulatorMotor(SysIdRoutineLog log) {
        log.motor("climber")
            .voltage(Units.Volts.of(manipulator_voltage_))
            .angularPosition(Units.Revolutions.of(manipulator_encoder_.getPosition()))
            .angularVelocity(Units.RevolutionsPerSecond.of(manipulator_encoder_.getVelocity())) ;
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // Climber
    //
    ////////////////////////////////////////////////////////////////////////////

    public void setClimberMotorVoltage(double volts) {
        climber_voltage_ = volts ;
        climber_motor_.setControl(new VoltageOut(climber_voltage_));
    }

    public double getClimberVoltage() {
        return climber_voltage_;
    }

    public void logClimberMotor(SysIdRoutineLog log) {
        log.motor("climber")
            .voltage(Units.Volts.of(climber_voltage_))
            .angularPosition(Units.Revolutions.of(climber_pos_sig_.refresh().getValueAsDouble()))
            .angularVelocity(Units.RevolutionsPerSecond.of(climber_velocity_sig_.refresh().getValueAsDouble())) ;
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // Utility methods
    //
    ////////////////////////////////////////////////////////////////////////////

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
        return motors_ ;
    }
}
