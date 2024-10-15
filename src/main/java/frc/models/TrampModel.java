package frc.models;

import java.util.Map;

import org.xero1425.base.ISubsystemSim;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TrampModel extends SimulationModel {

    private static final double kArmGearRation = 14.81 ;
    private static final double kArmInertia = 0.000001 ;    

    private static final double kElevatorGearRatio = 1.0 ;
    private static final double kElevatorInertia = 0.00001 ;      

    private static final double kClimberGearRatio = 250.0 ;
    private static final double kClimberInertia = 0.000001 ;     

    private DCMotorSim arm_sim_ ; ;
    private DCMotorSim elevator_sim_ ;
    private DCMotorSim climber_sim_ ;

    private TalonFX arm_ ;
    private TalonFX elevator_ ;
    private TalonFX climber_ ;

    public TrampModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception 
    {
        setCreated();        

        return true ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false ;
        
        try {
            if (name.equals("has-note")) {
                ISubsystemSim simsub = getEngine().getRobot().getSubsystemByName(TrampSubsystem.NAME) ;
                if (simsub instanceof TrampSubsystem) {
                    TrampSubsystem sub = (TrampSubsystem)simsub ;
                    sub.setHasNote(value.getBoolean()) ;
                }
                ret = true ;
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

    private boolean createModels() {
        SimulationEngine engine = getEngine() ;

        Map<String, TalonFX> motors = engine.getRobot().getSubsystemByName(TrampSubsystem.NAME).getCTREMotors() ;

        if (!motors.containsKey(TrampSubsystem.ARM_MOTOR_NAME)) {
            return false ;
        }
        arm_ = motors.get(TrampSubsystem.ARM_MOTOR_NAME) ;
        arm_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kArmGearRation, kArmInertia) ;

        if (!motors.containsKey(TrampSubsystem.ELEVATOR_MOTOR_NAME)) {
            return false ;
        }
        elevator_ = motors.get(TrampSubsystem.ELEVATOR_MOTOR_NAME) ;
        elevator_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kElevatorGearRatio, kElevatorInertia) ;

        if (!motors.containsKey(TrampSubsystem.CLIMBER_MOTOR_NAME)) {
            return false ;
        }

        climber_ = motors.get(TrampSubsystem.CLIMBER_MOTOR_NAME) ;
        climber_sim_= new DCMotorSim(DCMotor.getKrakenX60(1), kClimberGearRatio, kClimberInertia) ;

        return true;        
    }

    @Override
    public void run(double dt) {
        if (arm_ == null) {
            if (!createModels()) {
                MessageLogger logger = MessageLogger.getTheMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("time", getEngine().getSimulationTime());
                logger.add("msg", "failed to create intake-shooter motor models") ;
                logger.endMessage();
            }
        }        

        double battery = RobotController.getBatteryVoltage() ;
        if (arm_ != null && arm_sim_!= null) {
            TalonFXSimState st = arm_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            arm_sim_.setInputVoltage(mv) ;
            arm_sim_.update(dt) ;

            st.setRawRotorPosition(arm_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(arm_sim_.getAngularVelocityRadPerSec())) ;
        }

        if (elevator_ != null && elevator_sim_!= null) {
            TalonFXSimState st = elevator_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            elevator_sim_.setInputVoltage(mv) ;
            elevator_sim_.update(dt) ;

            st.setRawRotorPosition(elevator_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(elevator_sim_.getAngularVelocityRadPerSec())) ;
        }        

        if (climber_ != null && climber_sim_!= null) {
            TalonFXSimState st = climber_.getSimState() ;
            st.setSupplyVoltage(battery) ;

            double mv = st.getMotorVoltage() ;
            climber_sim_.setInputVoltage(mv) ;
            climber_sim_.update(dt) ;

            st.setRawRotorPosition(climber_sim_.getAngularPositionRotations()) ;
            st.setRotorVelocity(Units.radiansToRotations(climber_sim_.getAngularVelocityRadPerSec())) ;
        }        
    }
}
