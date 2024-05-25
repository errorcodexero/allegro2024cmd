package frc.robot.subsystems.tramp;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface TrampIO {

    @AutoLog
    public static class TrampIOInputs {
        public double elevatorPosition = 0.0 ;
        public double elevatorVelocity = 0.0 ;
        public double elevatorCurrent = 0.0 ;
        public double elevatorOutput = 0.0 ;
        public double elevatorEncoder = 0.0 ;
        public double armPosition = 0.0 ;
        public double armVelocity = 0.0 ;
        public double armCurrent = 0.0 ;
        public double armOutput = 0.0 ;
        public double armEncoder = 0.0 ;
        public double manipulatorPosition = 0.0 ;
        public double manipulatorVelocity = 0.0 ;
        public double manipulatorCurrent = 0.0 ;
        public double climberPosition = 0.0 ;
        public double climberCurrent = 0.0 ;
        public double climberVelocity = 0.0 ;
        public double climberOutput = 0.0 ;
        public double climberEncoder = 0.0 ;
    }

    public default void updateInputs(TrampIOInputs inputs) {
    }

    public default void setElevatorTargetPos(double pos) {
    }

    public default void setElevatorMotorVoltage(double volts) {
    }

    public default double getElevatorMotorVoltage() {
        return 0.0 ;
    }

    public default void logElevatorMotor(SysIdRoutineLog log) {
    }        

    public default void setArmTargetPos(double pos) {
    }

    public default void setArmPosition(double pos) {
    }

    public default void setArmMotorVoltage(double volts) {
    }

    public default double getArmMotorVoltage() {
        return 0.0 ;
    }    

    public default void logArmMotor(SysIdRoutineLog log) {
    }      

    public default void setManipulatorTargetPosition(double pos) {
    }

    public default void setManipulatorVoltage(double volts) {
    }

    public default double getManipulatorVoltage() {
        return 0.0 ;
    }

    public default void logManipulatorMotor(SysIdRoutineLog log) {
    }   

    public default void setClimberMotorVoltage(double volts) {
    }

    public default double getClimberMotorVoltage() {
        return 0.0 ;
    }

    public default void logClimberMotor(SysIdRoutineLog log) {
    }      

    public default void simulate(double period) {
    }    

    public default List<TalonFX> getCTREMotors() {
        return null ;
    }

    public default List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }     
}
