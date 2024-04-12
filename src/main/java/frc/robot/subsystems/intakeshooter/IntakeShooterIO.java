package frc.robot.subsystems.intakeshooter;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeShooterIO {
    @AutoLog
    public static class IntakeShooterIOInputs {
        public double updownPosition = 0.0;
        public double updownVelocity = 0.0;
        public double updownCurrent = 0.0;
        public double tiltPosition = 0.0;
        public double tiltVelocity = 0.0;
        public double tiltCurrent = 0.0;
        public double getTiltAbsoluteEncoderPosition = 0.0;
        public double feederCurrent = 0.0;
        public double shooter1Velocity = 0.0 ;
        public double shooter1Current = 0.0 ;
        public double shooter1Position = 0.0 ;
        public double shooter2Velocity = 0.0 ;
        public double shooter2Current = 0.0 ;
        public double shooter2Position = 0.0 ;
        public boolean risingEdge = false ;
        public boolean fallingEdge = false ;
        public boolean noteSensor = false ;
    }

    public default void updateInputs(IntakeShooterIOInputs inputs) {
    }

    public default double getTiltAbsoluteEncoderPosition() {
        return 0.0;
    }

    public default void setUpDownTargetPos(double pos) {
    }

    public default void setUpDownMotorPosition(double pos) {
    }

    public default void setTiltTargetPos(double pos) {
    }

    public default void setTiltMotorPosition(double pos) {
    }    

    public default void setShooter1Velocity(double vel) {
    }

    public default void setShooter1Voltage(double volts) {
    }

    public default void setShooter2Velocity(double vel) {
    }

    public default void setShooter2Voltage(double volts) {
    }

    public default void setFeederVoltage(double volts) {
    }

    public default void simulate(double period) {
    }    
}
