package frc.robot.subsystems.tramp;

import org.xero1425.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

public class TrampIOHardware implements TrampIO {
    private TalonFX elevator_motor_ ;
    private TalonFX arm_motor_ ;
    private CANSparkFlex manipulator_motor_ ;

    private StatusSignal<Double> elevator_pos_sig_ ;
    private StatusSignal<Double> elevator_vel_sig_ ;
    private StatusSignal<Double> elevator_current_sig_ ;
    private StatusSignal<Double> arm_pos_sig_ ;
    private StatusSignal<Double> arm_vel_sig_ ;
    private StatusSignal<Double> arm_current_sig_ ;

    public TrampIOHardware() throws Exception {
        elevator_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Elevator.kMotorId,
                                                                    false,
                                                                    TrampConstants.Elevator.kCurrentLimit);
        arm_motor_ = TalonFXFactory.getFactory().createTalonFX(TrampConstants.Arm.kMotorId,
                                                               false,
                                                               TrampConstants.Arm.kCurrentLimit);

        manipulator_motor_ = new CANSparkFlex(TrampConstants.Manipulator.kMotorId, CANSparkFlex.MotorType.kBrushless);
        manipulator_motor_.setSmartCurrentLimit(60) ;
        manipulator_motor_.setIdleMode(IdleMode.kBrake) ;

        elevator_pos_sig_ = elevator_motor_.getPosition() ;
        elevator_vel_sig_ = elevator_motor_.getVelocity() ;
        elevator_current_sig_ = elevator_motor_.getSupplyCurrent() ;

        arm_pos_sig_ = arm_motor_.getPosition() ;
        arm_vel_sig_ = arm_motor_.getVelocity() ;
        arm_current_sig_ = arm_motor_.getSupplyCurrent() ;

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                                elevator_pos_sig_,
                                elevator_vel_sig_,
                                elevator_current_sig_,
                                arm_pos_sig_,
                                arm_vel_sig_,
                                arm_current_sig_) ;

        elevator_motor_.optimizeBusUtilization() ;
        arm_motor_.optimizeBusUtilization() ;
    }

    public void updateInputs(TrampIOInputs inputs) {
        inputs.elevatorPosition = elevator_pos_sig_.getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;
        inputs.elevatorVelocity = elevator_vel_sig_.getValueAsDouble() * TrampConstants.Elevator.kMetersPerRev ;
        inputs.elevatorCurrent = elevator_motor_.getSupplyCurrent().getValueAsDouble() ;

        inputs.armPosition = arm_pos_sig_.getValueAsDouble() * TrampConstants.Arm.kDegreesPerRev ;
        inputs.armVelocity = arm_vel_sig_.getValueAsDouble() * TrampConstants.Arm.kDegreesPerRev ;
        inputs.armCurrent = arm_motor_.getSupplyCurrent().getValueAsDouble() ;

        inputs.manipulatorCurrent = manipulator_motor_.getOutputCurrent() ;
    }

    public void setElevatorTargetPos(double pos) {
        elevator_motor_.setControl(new PositionTorqueCurrentFOC(pos / TrampConstants.Elevator.kMetersPerRev)) ;
    }

    public void setArmTargetPos(double pos) {
        arm_motor_.setControl(new PositionTorqueCurrentFOC(pos / TrampConstants.Arm.kDegreesPerRev)) ;
    }

    public void setManipulatorVoltage(double volts) {
        manipulator_motor_.set(volts) ;
    }    
}
