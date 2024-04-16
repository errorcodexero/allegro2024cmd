package frc.robot.subsystems.tramp;

import org.xero1425.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import com.revrobotics.CANSparkBase.IdleMode;

public class TrampIOHardware implements TrampIO {
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
    private StatusSignal<Double> arm_pos_sig_ ;
    private StatusSignal<Double> arm_vel_sig_ ;
    private StatusSignal<Double> arm_current_sig_ ;
    private StatusSignal<Double> climber_current_sig_ ;
    private StatusSignal<Double> climber_pos_sig_ ;

    private double manipulator_voltage_ ;
    private double climber_voltage_ ;

    public TrampIOHardware() throws Exception {
        Slot0Configs cfg ;

        elevator_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Elevator.kMotorId,
                                                                    false,
                                                                    TrampConstants.Elevator.kCurrentLimit);
        elevator_motor_.getPosition().setUpdateFrequency(100) ;
        elevator_motor_.getVelocity().setUpdateFrequency(100) ;
        cfg = new Slot0Configs().withKP(TrampConstants.Elevator.PID.kP)
                                .withKI(TrampConstants.Elevator.PID.kI)
                                .withKD(TrampConstants.Elevator.PID.kD)
                                .withKV(TrampConstants.Elevator.PID.kV)
                                .withKA(TrampConstants.Elevator.PID.kA)
                                .withKG(TrampConstants.Elevator.PID.kG)
                                .withKS(TrampConstants.Elevator.PID.kS) ;
        elevator_motor_.getConfigurator().apply(cfg) ;

        arm_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Arm.kMotorId,
                                                               false,
                                                               TrampConstants.Arm.kCurrentLimit);
        arm_motor_.getPosition().setUpdateFrequency(100) ;
        arm_motor_.getVelocity().setUpdateFrequency(100) ;
        cfg = new Slot0Configs().withKP(TrampConstants.Arm.PID.kP)
                                .withKI(TrampConstants.Arm.PID.kI)
                                .withKD(TrampConstants.Arm.PID.kD)
                                .withKV(TrampConstants.Arm.PID.kV)
                                .withKA(TrampConstants.Arm.PID.kA)
                                .withKG(TrampConstants.Arm.PID.kG)
                                .withKS(TrampConstants.Arm.PID.kS) ;      
        arm_motor_.getConfigurator().apply(cfg) ;

        climber_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Climber.kMotorId,
                                                               false,
                                                               TrampConstants.Climber.kCurrentLimit);
        climber_motor_.getPosition().setUpdateFrequency(100) ;
        climber_motor_.getVelocity().setUpdateFrequency(100) ;

        manipulator_motor_ = new CANSparkFlex(TrampConstants.Manipulator.kMotorId, CANSparkFlex.MotorType.kBrushless);
        manipulator_motor_.setSmartCurrentLimit(60) ;
        manipulator_motor_.setIdleMode(IdleMode.kBrake) ;
        encoder_ = manipulator_motor_.getEncoder() ;

        elevator_pos_sig_ = elevator_motor_.getPosition() ;
        elevator_vel_sig_ = elevator_motor_.getVelocity() ;
        elevator_current_sig_ = elevator_motor_.getSupplyCurrent() ;

        arm_pos_sig_ = arm_motor_.getPosition() ;
        arm_vel_sig_ = arm_motor_.getVelocity() ;
        arm_current_sig_ = arm_motor_.getSupplyCurrent() ;

        climber_pos_sig_ = climber_motor_.getPosition() ;
        climber_current_sig_ = climber_motor_.getSupplyCurrent() ;

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                                elevator_pos_sig_,
                                elevator_vel_sig_,
                                elevator_current_sig_,
                                arm_pos_sig_,
                                arm_vel_sig_,
                                arm_current_sig_,
                                climber_current_sig_) ;

        elevator_motor_.optimizeBusUtilization() ;
        arm_motor_.optimizeBusUtilization() ;
        climber_motor_.optimizeBusUtilization() ;

        manipulator_voltage_ = 0.0 ;
        climber_voltage_ = 0.0 ;

        if (RobotBase.isSimulation()) {
            arm_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), TrampConstants.Arm.kSimGearRatio, TrampConstants.Arm.kSimMotorLoad) ;
            elevator_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), TrampConstants.Elevator.kSimGearRatio, TrampConstants.Elevator.kSimMotorLoad);
            climber_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), TrampConstants.Climber.kSimGearRatio, TrampConstants.Climber.kSimMotorLoad) ;
        }        
    }

    public void updateInputs(TrampIOInputs inputs) {
        inputs.elevatorPosition = elevator_pos_sig_.refresh().getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;
        inputs.elevatorVelocity = elevator_vel_sig_.refresh().getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;
        inputs.elevatorCurrent = elevator_current_sig_.refresh().getValueAsDouble() ;

        inputs.armPosition = arm_pos_sig_.refresh().getValueAsDouble() * TrampConstants.Arm.kDegreesPerRev ;
        inputs.armVelocity = arm_vel_sig_.refresh().getValueAsDouble() * TrampConstants.Arm.kDegreesPerRev ;
        inputs.armCurrent = arm_current_sig_.refresh().getValueAsDouble() ;

        inputs.climberCurrent = climber_current_sig_.refresh().getValueAsDouble() ;
        inputs.climberPositon = climber_pos_sig_.refresh().getValueAsDouble() ;

        inputs.manipulatorPosition = encoder_.getPosition() ;
        inputs.manipulatorCurrent = manipulator_motor_.getOutputCurrent() ;        
    }

    public void setElevatorTargetPos(double pos) {
        elevator_motor_.setControl(new PositionTorqueCurrentFOC(pos / TrampConstants.Elevator.kMetersPerRev)) ;
    }

    public void setArmTargetPos(double pos) {
        arm_motor_.setControl(new PositionTorqueCurrentFOC(pos / TrampConstants.Arm.kDegreesPerRev)) ;
    }

    public void setManipulatorVoltage(double volts) {
        manipulator_voltage_ = volts ;
        manipulator_motor_.set(manipulator_voltage_) ;
    }

    public double getManipulatorVoltage() {
        return manipulator_voltage_ ;
    }

    public void setClimberVoltage(double volts) {
        climber_voltage_ = volts ;
        climber_motor_.setControl(new VoltageOut(climber_voltage_));
    }

    public double getClimberVoltage() {
        return climber_voltage_;
    }    

    public void doSim(TalonFX motor, DCMotorSim sim, double period){
        TalonFXSimState state = motor.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;
        sim.setInputVoltage(state.getMotorVoltage());
        sim.update(period) ;
        state.setRawRotorPosition(sim.getAngularPositionRotations()) ;
        state.setRotorVelocity(Units.radiansToRotations(sim.getAngularVelocityRadPerSec())) ;
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
}
