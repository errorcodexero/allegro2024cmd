package frc.robot.subsystems.tramp;

import java.util.function.Supplier;

import org.xero1425.TalonFXFactory;

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
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units ;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import com.revrobotics.CANSparkBase.IdleMode;

public class TrampIOHardware implements TrampIO {
    private final static int kApplyTries = 5 ;

    private TalonFX elevator_motor_ ;
    private TalonFX arm_motor_ ;
    private TalonFX climber_motor_ ;
    private CANSparkFlex manipulator_motor_ ;
    private RelativeEncoder encoder_ ;    
    private DCMotorSim arm_sim_ ;
    private DCMotorSim elevator_sim_ ;
    private DCMotorSim climber_sim_ ;

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

    public TrampIOHardware() throws Exception {

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Elevator motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////            
        elevator_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Elevator.kMotorId,
                                                                    TrampConstants.Elevator.kInverted,
                                                                    TrampConstants.Elevator.kCurrentLimit);
                                                                    elevator_motor_.setPosition(0.0) ;
        final Slot0Configs elevatorcfg = new Slot0Configs().withKP(TrampConstants.Elevator.PID.kP)
                                .withKI(TrampConstants.Elevator.PID.kI)
                                .withKD(TrampConstants.Elevator.PID.kD)
                                .withKV(TrampConstants.Elevator.PID.kV)
                                .withKA(TrampConstants.Elevator.PID.kA)
                                .withKG(TrampConstants.Elevator.PID.kG)
                                .withKS(TrampConstants.Elevator.PID.kS)
                                .withGravityType(GravityTypeValue.Elevator_Static) ;
        checkError("set-elevator-PID-value", () -> elevator_motor_.getConfigurator().apply(elevatorcfg)) ;

        final MotionMagicConfigs mmcfg = new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(TrampConstants.Elevator.MotionMagic.kMaxVelocity)
                            .withMotionMagicAcceleration(TrampConstants.Elevator.MotionMagic.kMaxAcceleration)
                            .withMotionMagicJerk(TrampConstants.Elevator.MotionMagic.kJerk) ;
        checkError("set-elevator-motion-magic", () -> elevator_motor_.getConfigurator().apply(mmcfg)) ;

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
        final Slot0Configs armcfg = new Slot0Configs().withKP(TrampConstants.Arm.PID.kP)
                                .withKI(TrampConstants.Arm.PID.kI)
                                .withKD(TrampConstants.Arm.PID.kD)
                                .withKV(TrampConstants.Arm.PID.kV)
                                .withKA(TrampConstants.Arm.PID.kA)
                                .withKG(TrampConstants.Arm.PID.kG)
                                .withKS(TrampConstants.Arm.PID.kS)
                                .withGravityType(GravityTypeValue.Arm_Cosine) ;
        checkError("set-arm-PID-value", () -> arm_motor_.getConfigurator().apply(armcfg)) ;

        final MotionMagicConfigs armmmcfg = new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(TrampConstants.Arm.MotionMagic.kMaxVelocity)
                            .withMotionMagicAcceleration(TrampConstants.Arm.MotionMagic.kMaxAcceleration)
                            .withMotionMagicJerk(TrampConstants.Arm.MotionMagic.kJerk) ;
        checkError("set-elevator-motion-magic", () -> arm_motor_.getConfigurator().apply(armmmcfg)) ;

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
        climber_pos_sig_ = climber_motor_.getPosition() ;
        climber_current_sig_ = climber_motor_.getSupplyCurrent() ;
        climber_output_sig_ = climber_motor_.getClosedLoopOutput();
        climber_velocity_sig_ = climber_motor_.getVelocity() ;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Manipulator motor initialization
        /////////////////////////////////////////////////////////////////////////////////////////////////           
        manipulator_motor_ = new CANSparkFlex(TrampConstants.Manipulator.kMotorId, CANSparkFlex.MotorType.kBrushless);
        manipulator_motor_.setInverted(TrampConstants.Manipulator.kInverted);
        manipulator_motor_.setSmartCurrentLimit(60) ;
        manipulator_motor_.setIdleMode(IdleMode.kBrake) ;
        encoder_ = manipulator_motor_.getEncoder() ;
        encoder_.setPositionConversionFactor(1.0 / (double)encoder_.getCountsPerRevolution()) ;
        encoder_.setVelocityConversionFactor(1.0 / (double)encoder_.getCountsPerRevolution()) ;        

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

        manipulator_voltage_ = 0.0 ;
        climber_voltage_ = 0.0 ;

        if (RobotBase.isSimulation()) {
            arm_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), TrampConstants.Arm.kSimGearRatio, TrampConstants.Arm.kSimMotorLoad) ;
            elevator_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), TrampConstants.Elevator.kSimGearRatio, TrampConstants.Elevator.kSimMotorLoad);
            climber_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), TrampConstants.Climber.kSimGearRatio, TrampConstants.Climber.kSimMotorLoad) ;
        }        
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
        inputs.climberPositon = enc ;        
        inputs.climberCurrent = climber_current_sig_.refresh().getValueAsDouble() ;
        inputs.climberOutput = climber_output_sig_.refresh().getValueAsDouble() ;
        inputs.climberVelocity = climber_velocity_sig_.refresh().getValueAsDouble() ;
        inputs.climberEncoder = enc ;

        inputs.manipulatorPosition = encoder_.getPosition() ;
        inputs.manipulatorVelocity = encoder_.getVelocity() ;
        inputs.manipulatorCurrent = manipulator_motor_.getOutputCurrent() ;        
    }

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

    public void setManipulatorVoltage(double volts) {
        manipulator_voltage_ = volts ;
        manipulator_motor_.setVoltage(volts);
    }

    public double getManipulatorVoltage() {
        return manipulator_voltage_ ;
    }

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
    
    public void logManipulatorMotor(SysIdRoutineLog log) {
        log.motor("climber")
            .voltage(Units.Volts.of(manipulator_voltage_))
            .angularPosition(Units.Revolutions.of(encoder_.getPosition()))
            .angularVelocity(Units.RevolutionsPerSecond.of(encoder_.getVelocity())) ;
    }      

    public void doSim(TalonFX motor, DCMotorSim sim, double period){
        TalonFXSimState state = motor.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;
        sim.setInputVoltage(state.getMotorVoltage());
        sim.update(period) ;
        state.setRawRotorPosition(sim.getAngularPositionRotations()) ;
        state.setRotorVelocity(edu.wpi.first.math.util.Units.radiansToRotations(sim.getAngularVelocityRadPerSec())) ;
    }    

    public void simulate(double period) {
        doSim(arm_motor_, arm_sim_, period) ;
        doSim(elevator_motor_, elevator_sim_, period) ;
        doSim(climber_motor_, climber_sim_, period) ;

        //
        // The RevRobotics motors do not provide simulation support, so we have to simulate them
        // here "manually".  This simulation is specific to the use case here
        //
        if (Math.abs(manipulator_voltage_) > 0.01) {
            double pos = encoder_.getPosition() + 0.04 ;
            encoder_.setPosition(pos) ;
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
}
