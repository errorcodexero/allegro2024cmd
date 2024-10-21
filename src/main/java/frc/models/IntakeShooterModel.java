package frc.models;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;

public class IntakeShooterModel extends SimulationModel {
    private static final double kShooterGearRatio = 1.0 / 1.0 ;
    private static final double kShooterInertia = 0.000001 ;

    private static final double kFeederGearRatio = 1.0 / 1.0 ;
    private static final double kFeederInertia = 0.000001 ;
    
    private static final double kUpDownGearRatio = 2 ;
    private static final double kUpDownInertia = 0.000001 ;

    private static final double kTiltGearRatio = 2.0 ;
    private static final double kTiltInertia = 0.000001 ;

    private DCMotorSim shooter1_sim_ ;
    private DCMotorSim shooter2_sim_ ;
    private DCMotorSim feeder_sim_ ;
    private DCMotorSim updown_sim_ ;
    private DCMotorSim tilt_sim_ ;

    private TalonFX shooter1_ ;
    private TalonFX shooter2_ ;

    private TalonFX feeder_ ;
    private TalonFX updown_ ;
    private TalonFX tilt_ ;

    int note_sensor_ ;
    
    public IntakeShooterModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        note_sensor_ = getProperty("note-sensor").getInteger() ;
        DIODataJNI.setIsInput(note_sensor_, true);
        DIODataJNI.setValue(note_sensor_, true) ;

        setCreated();

        return true ;
    }

    private boolean createModels() {
        SimulationEngine engine = getEngine() ;

        Map<String, TalonFX> motors = engine.getRobot().getSubsystemByName(IntakeShooterSubsystem.NAME).getCTREMotors() ;

        if (!motors.containsKey(IntakeShooterSubsystem.FEEDER_MOTOR_NAME)) {
            return false ;
        }
        feeder_ = motors.get(IntakeShooterSubsystem.FEEDER_MOTOR_NAME) ;
        feeder_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kFeederGearRatio, kFeederInertia) ;

        if (!motors.containsKey(IntakeShooterSubsystem.SHOOTER1_MOTOR_NAME)) {
            return false ;
        }
        shooter1_ = motors.get(IntakeShooterSubsystem.SHOOTER1_MOTOR_NAME) ;
        shooter1_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kShooterGearRatio, kShooterInertia) ;

        if (!motors.containsKey(IntakeShooterSubsystem.SHOOTER2_MOTOR_NAME)) {
            return false ;
        }
        shooter2_ = motors.get(IntakeShooterSubsystem.SHOOTER2_MOTOR_NAME) ;
        shooter2_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kShooterGearRatio, kShooterInertia) ;

        if (!motors.containsKey(IntakeShooterSubsystem.UPDOWN_MOTOR_NAME)) {
            return false ;
        }
        updown_ = motors.get(IntakeShooterSubsystem.UPDOWN_MOTOR_NAME) ;
        updown_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kUpDownGearRatio, kUpDownInertia) ;

        if (!motors.containsKey(IntakeShooterSubsystem.TILT_MOTOR_NAME)) {
            return false ;
        }
        tilt_ = motors.get(IntakeShooterSubsystem.TILT_MOTOR_NAME) ;
        tilt_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kTiltGearRatio, kTiltInertia) ;

        return true;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false ;
        
        try {
            if (name.equals("note-sensor")) {
                DIODataJNI.setIsInput(note_sensor_, false);
                DIODataJNI.setValue(note_sensor_, value.getBoolean());
            }
        }
        catch(Exception ex) {
            MessageLogger logger = MessageLogger.getTheMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("time", getEngine().getSimulationTime());
            logger.add("event", name) ;
            logger.add("- expected boolean value, but got " + value.toString()) ;
            logger.endMessage();
        }
        return ret ;
    }

    @Override
    public void run(double dt) {
        if (shooter1_ == null) {
            if (!createModels()) {
                MessageLogger logger = MessageLogger.getTheMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("time", getEngine().getSimulationTime());
                logger.add("msg", "failed to create intake-shooter motor models") ;
                logger.endMessage();
            }
        }

        double battery = RobotController.getBatteryVoltage() ;
        Logger.recordOutput("battery", battery) ;
        
        if (shooter1_ != null && shooter1_sim_!= null) {
            TalonFXSimState st = shooter1_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            shooter1_sim_.setInputVoltage(mv) ;
            shooter1_sim_.update(dt) ;

            st.setRawRotorPosition(shooter1_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(shooter1_sim_.getAngularVelocityRadPerSec())) ;
        }

        if (shooter2_ != null && shooter2_sim_!= null) {
            TalonFXSimState st = shooter2_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            shooter2_sim_.setInputVoltage(mv) ;
            shooter2_sim_.update(dt) ;

            st.setRawRotorPosition(shooter2_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(shooter2_sim_.getAngularVelocityRadPerSec())) ;
        }

        if (feeder_ != null && feeder_sim_!= null) {
            TalonFXSimState st = feeder_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            feeder_sim_.setInputVoltage(mv) ;
            feeder_sim_.update(dt) ;

            st.setRawRotorPosition(feeder_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(feeder_sim_.getAngularVelocityRadPerSec())) ;
        }

        if (updown_ != null && updown_sim_!= null) {
            TalonFXSimState st = updown_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            updown_sim_.setInputVoltage(mv) ;
            updown_sim_.update(dt) ;


            double pos = updown_sim_.getAngularPositionRotations() ;
            st.setRawRotorPosition(pos) ;
            st.setRotorVelocity(Units.radiansToRotations(updown_sim_.getAngularVelocityRadPerSec())) ;    
        }

        if (tilt_ != null && tilt_sim_!= null) {
            TalonFXSimState st = tilt_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            tilt_sim_.setInputVoltage(mv) ;
            tilt_sim_.update(dt) ;

            st.setRawRotorPosition(tilt_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(tilt_sim_.getAngularVelocityRadPerSec())) ;
        }
    }
}
