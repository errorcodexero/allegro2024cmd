package frc.robot.subsystems.tramp;

import org.littletonrobotics.junction.AutoLog;

public interface TrampIO {

    @AutoLog
    public static class TrampIOInputs {
        public double elevatorPosition = 0.0 ;
        public double elevatorVelocity = 0.0 ;
        public double elevatorCurrent = 0.0 ;
        public double armPosition = 0.0 ;
        public double armVelocity = 0.0 ;
        public double armCurrent = 0.0 ;
        public double manipulatorPosition = 0.0 ;
        public double manipulatorCurrent = 0.0 ;
        public double climberPositon = 0.0 ;
        public double climberCurrent = 0.0 ;
    }

    public default void updateInputs(TrampIOInputs inputs) {
    }

    public default void setElevatorTargetPos(double pos) {
    }

    public default void setArmTargetPos(double pos) {
    }

    public default void setManipulatorVoltage(double volts) {
    }

    public default double getManipulatorVoltage() {
        return 0.0 ;
    }

    public default void setClimberVoltage(double volts) {
    }

    public default double getClimberVoltage() {
        return 0.0 ;
    }

    public default void simulate(double period) {
    }    
}
