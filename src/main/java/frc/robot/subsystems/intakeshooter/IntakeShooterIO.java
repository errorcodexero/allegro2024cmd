package frc.robot.subsystems.intakeshooter;

import java.util.Map;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface IntakeShooterIO {
    @AutoLog
    public static class IntakeShooterIOInputs {
        public double updownPosition = 0.0;
        public double updownVelocity = 0.0;
        public double updownCurrent = 0.0;
        public double updownVoltage = 0.0 ;
        public double updownEncoder = 0.0 ;
        public double tiltPosition = 0.0;
        public double tiltVelocity = 0.0;
        public double tiltAverageVelocity = 0.0 ;
        public double tiltCurrent = 0.0;
        public double tiltVoltage = 0.0 ;
        public double tiltEncoder = 0.0 ;
        public double tiltAbsoluteEncoderPosition = 0.0;
        public double tiltAbsoluteEncoderPositionMedian = 0.0 ;
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

    public default double getShooterPositionAtRisingEdge() {
        return 0.0 ;
    }

    public default void syncTiltEncoders(boolean b) {
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

    public default void setTiltTargetPos(double pos) {
    }

    public default void setTiltMotorPosition(double pos) {
    }

    public default void setTiltMotorVoltage(double vol) {
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

    public default double getFeederMotorVoltage() {
        return 0.0 ;
    }

    public default Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    public default List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    } 
}
