package frc.robot.subsystems.intakeshooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface IntakeShooterIO {
    @AutoLog
    public static class IntakeShooterIOInputs {
        public double updownPosition = 0.0;
        public double updownVelocity = 0.0;
        public double updownCurrent = 0.0;
        public double updownVoltage = 0.0 ;
        public double tiltPosition = 0.0;
        public double tiltVelocity = 0.0;
        public double tiltCurrent = 0.0;
        public double tiltVoltage = 0.0 ;
        public double tiltEncoder = 0.0 ;
        public double tiltAbsoluteEncoderPosition = 0.0;
        public double feederCurrent = 0.0;
        public double shooter1Velocity = 0.0 ;
        public double shooter1Current = 0.0 ;
        public double shooter1Position = 0.0 ;
        public double shooter1Voltage = 0.0 ;
        public double shooter2Velocity = 0.0 ;
        public double shooter2Current = 0.0 ;
        public double shooter2Position = 0.0 ;
        public double shooter2Voltage = 0.0 ;
        public boolean risingEdge = false ;
        public boolean fallingEdge = false ;
        public boolean noteSensor = false ;
    }

    public default void updateInputs(IntakeShooterIOInputs inputs) {
    }


    public default void setUpDownTargetPos(double pos) {
    }

    public default void setUpDownMotorPosition(double pos) {
    }

    public default void setUpDownMotorVoltage(double vol) {
    }    

    public default void logUpdownMotor(SysIdRoutineLog log) {
    }    

    public default void setTiltTargetPos(boolean tracking, double pos) {
    }

    public default void setTiltMotorPosition(double pos) {
    }

    public default void setTiltMotorVoltage(double vol) {
    }

    public default double getTiltAbsoluteEncoderPosition() {
        return 0.0;
    }    

    public default void setTiltMovementPID() throws Exception{
    }

    public default void setTiltTrackingPID() throws Exception{
    }

    public default void logTiltMotor(SysIdRoutineLog log) {
    }

    public default void setShooter1Velocity(double vel) {
    }

    public default void setShooter1MotorVoltage(double vol) {
    }    

    public default void logShooter1Motor(SysIdRoutineLog log) {
    }    

    public default void setShooter2Velocity(double vel) {
    }

    public default void setShooter2MotorVoltage(double vol) {
    }       

    public default void logShooter2Motor(SysIdRoutineLog log) {
    }    

    public default void setFeederMotorVoltage(double volts) {
    }

    public default void simulate(double period) {
    }    
}
