package frc.robot.subsystems.intakeshooter;

import org.littletonrobotics.junction.Logger;
import org.xero1425.EncoderMapper;
import org.xero1425.TalonFXFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeShooterIOTalonFX implements IntakeShooterIO {
    private TalonFX feeder_motor_ ;
    private TalonFX updown_motor_ ;
    private TalonFX shooter1_motor_ ;
    private TalonFX shooter2_motor_ ;
    private TalonFX tilt_motor_ ;
    private DigitalInput note_sensor_ ;    
    private AnalogInput absolute_encoder_;
    private AsynchronousInterrupt note_interrupt_ ;
    private EncoderMapper encoder_mapper_ ;
    private boolean rising_seen_ ;
    private boolean falling_seen_ ;
    private DCMotorSim updown_sim_ ;
    private DCMotorSim tilt_sim_ ;

    private StatusSignal<Double> updown_position_signal_ ;
    private StatusSignal<Double> updown_velocity_signal_ ;
    private StatusSignal<Double> updown_current_signal_ ;
    private StatusSignal<Double> tilt_position_signal_ ;
    private StatusSignal<Double> tilt_velocity_signal_ ;
    private StatusSignal<Double> tilt_current_signal_ ;
    private StatusSignal<Double> feeder_current_signal_ ;
    private StatusSignal<Double> shooter1_velocity_signal_ ;
    private StatusSignal<Double> shooter1_current_signal_ ;
    private StatusSignal<Double> shooter2_velocity_signal_ ;
    private StatusSignal<Double> shooter2_current_signal_ ;

    public IntakeShooterIOTalonFX(boolean practice) throws Exception {
        Slot0Configs cfg ;

        feeder_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Feeder.kMotorId,
                    false,
                    IntakeShooterConstants.Feeder.kCurrentLimit);

        updown_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.UpDown.kMotorId,
                    false,
                    IntakeShooterConstants.UpDown.kCurrentLimit);
        updown_motor_.getPosition().setUpdateFrequency(100) ;
        updown_motor_.getVelocity().setUpdateFrequency(100) ;
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.UpDown.PID.kP)
                                .withKI(IntakeShooterConstants.UpDown.PID.kI)
                                .withKD(IntakeShooterConstants.UpDown.PID.kD)
                                .withKV(IntakeShooterConstants.UpDown.PID.kV)
                                .withKA(IntakeShooterConstants.UpDown.PID.kA)
                                .withKG(IntakeShooterConstants.UpDown.PID.kG)
                                .withKS(IntakeShooterConstants.UpDown.PID.kS) ;
        updown_motor_.getConfigurator().apply(cfg) ;

                    
        tilt_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Tilt.kMotorId,
                    false,
                    IntakeShooterConstants.Tilt.kCurrentLimit);
        tilt_motor_.getPosition().setUpdateFrequency(100) ;
        tilt_motor_.getVelocity().setUpdateFrequency(100) ;        
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.Tilt.PID.kP)
                                .withKI(IntakeShooterConstants.Tilt.PID.kI)
                                .withKD(IntakeShooterConstants.Tilt.PID.kD)
                                .withKV(IntakeShooterConstants.Tilt.PID.kV)
                                .withKA(IntakeShooterConstants.Tilt.PID.kA)
                                .withKG(IntakeShooterConstants.Tilt.PID.kG)
                                .withKS(IntakeShooterConstants.Tilt.PID.kS) ;
        tilt_motor_.getConfigurator().apply(cfg) ;

        shooter1_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter1.kMotorId,
                    false,
                    IntakeShooterConstants.Shooter1.kCurrentLimit);
        shooter1_motor_.getVelocity().setUpdateFrequency(100) ;
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.Shooter.kP)
                                .withKI(IntakeShooterConstants.Shooter.kI)
                                .withKD(IntakeShooterConstants.Shooter.kD)
                                .withKV(IntakeShooterConstants.Shooter.kV)
                                .withKA(IntakeShooterConstants.Shooter.kA)
                                .withKG(IntakeShooterConstants.Shooter.kG)
                                .withKS(IntakeShooterConstants.Shooter.kS) ;
        shooter1_motor_.getConfigurator().apply(cfg) ;

        shooter2_motor_ = TalonFXFactory.getFactory().createTalonFX(
                    IntakeShooterConstants.Shooter2.kMotorId,
                    false,
                    IntakeShooterConstants.Shooter2.kCurrentLimit);
        shooter2_motor_.getVelocity().setUpdateFrequency(100) ;                    
        cfg = new Slot0Configs().withKP(IntakeShooterConstants.Shooter.kP)
                                .withKI(IntakeShooterConstants.Shooter.kI)
                                .withKD(IntakeShooterConstants.Shooter.kD)
                                .withKV(IntakeShooterConstants.Shooter.kV)
                                .withKA(IntakeShooterConstants.Shooter.kA)
                                .withKG(IntakeShooterConstants.Shooter.kG)
                                .withKS(IntakeShooterConstants.Shooter.kS) ;
        shooter2_motor_.getConfigurator().apply(cfg) ;

        note_sensor_ = new DigitalInput(IntakeShooterConstants.NoteSensor.kChannel) ;
        note_interrupt_ = new AsynchronousInterrupt(note_sensor_, (rising, falling) -> { noteInterruptHandler(rising, falling); }) ;
        note_interrupt_.setInterruptEdges(true, true);            
        note_interrupt_.enable();

        absolute_encoder_ = new AnalogInput(IntakeShooterConstants.Tilt.AbsoluteEncoder.kChannel) ;
        encoder_mapper_ = new EncoderMapper(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotMin,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMax,
                                            IntakeShooterConstants.Tilt.AbsoluteEncoder.kEncoderMin) ;

        if (practice) {
            encoder_mapper_.calibrate(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotCalibrationValue,
                                        IntakeShooterConstants.Tilt.AbsoluteEncoder.Practice.kEncoderCalibrationValue) ;
        }
        else {
            encoder_mapper_.calibrate(IntakeShooterConstants.Tilt.AbsoluteEncoder.kRobotCalibrationValue,
                                        IntakeShooterConstants.Tilt.AbsoluteEncoder.Competition.kEncoderCalibrationValue) ;            
        }       
        
        updown_position_signal_ = updown_motor_.getPosition() ;
        updown_velocity_signal_ = updown_motor_.getVelocity() ;
        updown_current_signal_ = updown_motor_.getSupplyCurrent() ;
        tilt_position_signal_ = tilt_motor_.getPosition() ;
        tilt_velocity_signal_ = tilt_motor_.getVelocity() ;
        tilt_current_signal_ = tilt_motor_.getSupplyCurrent() ;
        feeder_current_signal_ = feeder_motor_.getSupplyCurrent() ;
        shooter1_velocity_signal_ = shooter1_motor_.getVelocity() ;
        shooter1_current_signal_ = shooter1_motor_.getSupplyCurrent() ;
        shooter2_velocity_signal_ = shooter2_motor_.getVelocity() ;
        shooter2_current_signal_ = shooter2_motor_.getSupplyCurrent() ;  
        
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                                                    updown_position_signal_,
                                                    updown_velocity_signal_,
                                                    updown_current_signal_,
                                                    tilt_position_signal_,
                                                    tilt_velocity_signal_,
                                                    tilt_current_signal_,
                                                    feeder_current_signal_,
                                                    shooter1_velocity_signal_,
                                                    shooter1_current_signal_,
                                                    shooter2_velocity_signal_,
                                                    shooter2_current_signal_) ;

        updown_motor_.optimizeBusUtilization();
        tilt_motor_.optimizeBusUtilization();
        shooter1_motor_.optimizeBusUtilization();
        shooter2_motor_.optimizeBusUtilization();
        feeder_motor_.optimizeBusUtilization();

        if (RobotBase.isSimulation()) {
            updown_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 24.0, 0.001) ;
            tilt_sim_ = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 18.0, 0.001) ;
        }        
    }    

    public void updateInputs(IntakeShooterIOInputs inputs) {
        inputs.updownPosition = updown_position_signal_.getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
        inputs.updownVelocity = updown_velocity_signal_.getValueAsDouble() * IntakeShooterConstants.UpDown.kDegreesPerRev ;
        inputs.updownCurrent = updown_current_signal_.getValueAsDouble() ;

        inputs.tiltPosition = tilt_position_signal_.getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
        inputs.tiltVelocity = tilt_velocity_signal_.getValueAsDouble() * IntakeShooterConstants.Tilt.kDegreesPerRev ;
        inputs.tiltCurrent = tilt_current_signal_.getValueAsDouble() ;

        inputs.getTiltAbsoluteEncoderPosition = getTiltAbsoluteEncoderPosition() ;

        inputs.feederCurrent = feeder_current_signal_.getValueAsDouble() ;

        inputs.shooter1Velocity = shooter1_velocity_signal_.getValueAsDouble() ;
        inputs.shooter1Current = shooter1_current_signal_.getValueAsDouble() ;
        inputs.shooter1Position = shooter1_motor_.getPosition().getValueAsDouble() ;

        inputs.shooter2Velocity = shooter2_velocity_signal_.getValueAsDouble() ;
        inputs.shooter2Current = shooter2_current_signal_.getValueAsDouble() ;
        inputs.shooter2Position = shooter2_motor_.getPosition().getValueAsDouble() ;

        inputs.risingEdge = rising_seen_ ;
        inputs.fallingEdge = falling_seen_ ;

        inputs.noteSensor = note_sensor_.get() ^ IntakeShooterConstants.NoteSensor.kInverted ;
    }

    public double getTiltAbsoluteEncoderPosition() {
        return encoder_mapper_.toRobot(absolute_encoder_.getVoltage()) ;
    }    

    public void setUpDownTargetPos(double t) {
        updown_motor_.setControl(new PositionTorqueCurrentFOC(t / IntakeShooterConstants.UpDown.kDegreesPerRev)) ;
    }

    public void setUpDownMotorPosition(double pos) {
        updown_motor_.setPosition(pos / IntakeShooterConstants.UpDown.kDegreesPerRev) ;
    }

    public void setTiltTargetPos(double t) {
        tilt_motor_.setControl(new PositionTorqueCurrentFOC(t / IntakeShooterConstants.Tilt.kDegreesPerRev)) ;
    }    

    public void setTiltMotorPosition(double pos) {
        tilt_motor_.setPosition(pos / IntakeShooterConstants.Tilt.kDegreesPerRev) ;
    }
 
    public void setShooter1Velocity(double vel) {
        shooter1_motor_.setControl(new VelocityTorqueCurrentFOC(vel)) ;
    }

    public void setShooter1Voltage(double v) {
        shooter2_motor_.setControl(new VoltageOut(v)) ;
    }

    public void setShooter2Velocity(double vel) {
        shooter2_motor_.setControl(new VelocityTorqueCurrentFOC(vel)) ;
    }

    public void setShooter2Voltage(double v) {
        shooter2_motor_.setControl(new VoltageOut(v)) ;
    }

    public void setFeederVoltage(double v) {
        feeder_motor_.setControl(new VoltageOut(v)) ;
    }    
 
    public void simulate(double period) {
        TalonFXSimState state ;

        state = tilt_motor_.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        Logger.recordOutput("battery-voltage", RobotController.getBatteryVoltage()) ;

        tilt_sim_.setInputVoltage(state.getMotorVoltage());
        tilt_sim_.update(period);
        Logger.recordOutput("tilt-voltage", state.getMotorVoltage()) ;
        state.setRawRotorPosition(tilt_sim_.getAngularPositionRotations()) ;
        state.setRotorVelocity(Units.radiansToRotations(tilt_sim_.getAngularVelocityRadPerSec())) ;

        state = updown_motor_.getSimState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage()) ;

        updown_sim_.setInputVoltage(state.getMotorVoltage());
        updown_sim_.update(period);
        Logger.recordOutput("updown-voltage", state.getMotorVoltage()) ;        
        state.setRawRotorPosition(updown_sim_.getAngularPositionRotations()) ;
        state.setRotorVelocity(Units.radiansToRotations(updown_sim_.getAngularVelocityRadPerSec())) ;        
    }    
    
    private void noteInterruptHandler(boolean rising, boolean falling) {
        rising_seen_ = rising ;
        falling_seen_ =  falling ;
    }
}
