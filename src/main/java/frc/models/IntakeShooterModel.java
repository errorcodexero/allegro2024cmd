package frc.models;

import java.util.Map;

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
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;

public class IntakeShooterModel extends SimulationModel {
    private static final double kShooterGearRatio = 0.6 / 1.0 ;
    private static final double kShooterInertia = 0.00001 ;

    private FlywheelSim shooter1_flywheel_ ;
    private FlywheelSim shooter2_flywheel_ ;


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

        if (!motors.containsKey(IntakeShooterSubsystem.SHOOTER1_MOTOR_NAME)) {
            return false ;
        }
        shooter1_ = motors.get(IntakeShooterSubsystem.SHOOTER1_MOTOR_NAME) ;
        shooter1_flywheel_ = new FlywheelSim(DCMotor.getKrakenX60(1), kShooterGearRatio, kShooterInertia) ;

        if (!motors.containsKey(IntakeShooterSubsystem.SHOOTER2_MOTOR_NAME)) {
            return false ;
        }
        shooter2_ = motors.get(IntakeShooterSubsystem.SHOOTER2_MOTOR_NAME) ;
        shooter2_flywheel_ = new FlywheelSim(DCMotor.getKrakenX60(1), kShooterGearRatio, kShooterInertia) ;

        if (!motors.containsKey(IntakeShooterSubsystem.UPDOWN_MOTOR_NAME)) {
            return false ;
        }
        updown_ = motors.get(IntakeShooterSubsystem.UPDOWN_MOTOR_NAME) ;

        if (!motors.containsKey(IntakeShooterSubsystem.TILT_MOTOR_NAME)) {
            return false ;
        }
        tilt_ = motors.get(IntakeShooterSubsystem.TILT_MOTOR_NAME) ;


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
            MessageLogger logger = getEngine().getRobot().getMessageLogger() ;
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
                MessageLogger logger = getEngine().getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("time", getEngine().getSimulationTime());
                logger.add("msg", "failed to create intake-shooter motor models") ;
                logger.endMessage();
            }
        }

        double battery = RobotController.getBatteryVoltage() ;
        if (shooter1_ != null) {
            TalonFXSimState st = shooter1_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            shooter1_flywheel_.setInputVoltage(mv) ;
            shooter1_flywheel_.update(dt) ;

            st.setRotorVelocity(kShooterGearRatio * Units.radiansToRotations(shooter1_flywheel_.getAngularVelocityRadPerSec())) ;
        }

        if (shooter2_ != null) {
            TalonFXSimState st = shooter2_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            shooter2_flywheel_.setInputVoltage(mv) ;
            shooter2_flywheel_.update(dt) ;

            st.setRotorVelocity(kShooterGearRatio * Units.radiansToRotations(shooter2_flywheel_.getAngularVelocityRadPerSec())) ;
        }        
    }
}
