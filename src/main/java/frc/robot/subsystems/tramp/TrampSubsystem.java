package frc.robot.subsystems.tramp;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;
import org.xero1425.XeroTimer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.NoteDestination;

public class TrampSubsystem extends XeroSubsystem {

    private enum ClimberDir {
        Up,
        Down,
        None
    } ;

    private enum State {
        Idle,
        Eject,
        GotoDirectToTarget,
        CrossMinToMax,
        CrossMinToMaxWaitArm,
        CrossMaxToMin,
        CrossMaxToMinWaitArm,
        HoldingTransferPosition,
        HoldingAmpPosition,
        MoveNote,
        MovingNote,
        HoldingTrapPosition,
        Climbing,
        Trap1,
        StartDepositNote,
        DepositingNote,
        Trap3,
        Transferring,
        Shooting,
        BasicClimbMovingElevatorArm,
        BasicClimbMovingClimber,        
        BasicClimbReady,        
        BasicClimbDown
    }

    private TrampIO io_ ;
    private TrampIOInputsAutoLogged inputs_ ;

    private State next_state_ ;
    private State state_ ;

    public boolean has_note_  ;

    private double end_target_elev_ ;
    private double target_elev_ ;
    private double elevpostol_ ;
    private double elevveltol_ ;

    private double end_target_arm_ ;
    private double target_arm_ ;
    private double armpostol_ ;
    private double armveltol_ ;

    private Supplier<NoteDestination> destsupplier_ ;

    private XeroTimer eject_timer_ ;
    private XeroTimer shoot_timer_ ;
    private XeroTimer deposit_trap_timer_ ;

    private Trigger ready_for_amp_trigger_ ;
    private Trigger ready_for_trap_trigger_ ;
    private Trigger climber_down_trigger_ ;
    private Trigger basic_climb_ready_trigger_ ;

    private ClimberDir climber_dir_ ;
    private double climber_target_ ;
    private double manipulator_target_ ;

    public TrampSubsystem(XeroRobot robot, Supplier<NoteDestination> dest) throws Exception {
        super(robot, "trap-arm") ;

        io_ = new TrampIOHardware() ;
        inputs_ = new TrampIOInputsAutoLogged() ;

        io_.setArmPosition(TrampConstants.Arm.kMinPosition);

        destsupplier_ = dest ;

        eject_timer_ = new XeroTimer(robot, "tramp-eject", TrampConstants.Manipulator.kEjectTime) ;
        shoot_timer_ = new XeroTimer(robot, "tramp-shoot", TrampConstants.Manipulator.kShootTime) ;
        deposit_trap_timer_ = new XeroTimer(robot, "tramp-deposit", TrampConstants.Manipulator.kDepositTime) ;
                   
        ready_for_amp_trigger_ = new Trigger(() -> state_ == State.HoldingAmpPosition) ;
        ready_for_trap_trigger_ = new Trigger(() -> state_ == State.HoldingTrapPosition) ; 
        climber_down_trigger_ = new Trigger(()-> inputs_.climberPosition < 0.1) ;
        basic_climb_ready_trigger_ = new Trigger(()-> state_ == State.BasicClimbReady) ;
        
        state_ = State.Idle ;
        climber_dir_ = ClimberDir.None ;
        climber_target_ = 0.0 ;
    }

    public Trigger readyForAmp() {
        return ready_for_amp_trigger_;
    }

    public Trigger readyForTrap() {
        return ready_for_trap_trigger_;
    }    

    public Trigger isClimberDown() {
        return climber_down_trigger_ ;
    }

    public Trigger isBasicClimbReady() {
        return basic_climb_ready_trigger_ ;
    }

    public boolean isNoteDetected() {
        return inputs_.noteSensor ^ TrampConstants.NoteSensor.kInverted ;
    }
    
    public boolean isIdle() {
        return state_ == State.Idle ;
    }

    public boolean isInTransferPosition() {
        return state_ == State.HoldingTransferPosition ;
    }

    public boolean isInTrapPosition() {
        return state_ == State.HoldingTrapPosition ;
    }    

    public boolean isInAmpPosition() {
        return state_ == State.HoldingAmpPosition ;
    }

    public Command climberUpCmd() {
        Command cmd = new FunctionalCommand(
                                    () -> basicClimbPrep(),
                                    () -> {},
                                    (Boolean b) -> {},
                                    () -> isIdle()) ;
        cmd.setName("climberUp") ;        
        return cmd ;
    }

    public Command basicClimbCmd() {
        Command cmd = new FunctionalCommand(
                                    () -> basicClimbExec(),
                                    () -> {},
                                    (Boolean b) -> {},
                                    () -> isIdle()) ;
        cmd.setName("basicClimb") ;        
        return cmd ;
    }    

    public Command ejectCommand() {
        Command cmd = new FunctionalCommand(
                                    () -> eject(),
                                    () -> {},
                                    (Boolean b) -> {},
                                    () -> isIdle()) ;
        cmd.setName("eject") ;        
        return cmd ;
    }

    public Command turtleCommand() {
        Command cmd = new FunctionalCommand(
                                    () -> turtle(),
                                    () -> {},
                                    (Boolean b) -> {},
                                    () -> isIdle()) ;
        cmd.setName("turtle") ;
        return cmd ;
    }

    public Command shootCommand() {
        Command cmd = new FunctionalCommand(
                                    () -> shoot(),
                                    () -> {},
                                    (Boolean b) -> {},
                                    () -> isIdle()) ;
        cmd.setName("shoot") ;        
        return cmd ;
    }

    public Command trapCommand() {
        Command cmd = new FunctionalCommand(
                                    () -> trap(),
                                    () -> {},
                                    (Boolean b) -> {},
                                    () -> isIdle()) ;
        cmd.setName("trap") ;
        return cmd ;
    }

    private void basicClimbPrep() {
        gotoPosition(TrampConstants.Elevator.Positions.kBasicClimb, Double.NaN, Double.NaN, 
                     TrampConstants.Arm.Positions.kBasicClimb, Double.NaN, Double.NaN);

        next_state_ = State.BasicClimbMovingElevatorArm ;
    }

    private void basicClimbExec() {
        climberDown() ;
        state_ = State.BasicClimbDown ;
    }

    public void shoot() {
        if (isInAmpPosition()) {
            io_.setManipulatorVoltage(TrampConstants.Manipulator.kShootPower);
            shoot_timer_.start() ;
            state_ = State.Shooting ;
        }
    }

    public void trap() {
        if (isInTrapPosition()) {
            climberDown();
            state_ = State.Climbing ;
        }
    }

    public void moveToTransferPosition() {
        gotoPosition(TrampConstants.Elevator.Positions.kTransfer, Double.NaN, Double.NaN, TrampConstants.Arm.Positions.kTransfer, Double.NaN, Double.NaN);
        next_state_ = State.HoldingTransferPosition ;
    }

    public void moveToDestinationPosition() {
        NoteDestination dest = destsupplier_.get() ;
        if (dest == NoteDestination.Trap) {
            gotoPosition(TrampConstants.Elevator.Positions.kTrap, Double.NaN, Double.NaN, 
                         TrampConstants.Arm.Positions.kTrap, Double.NaN, Double.NaN);
            climberUp() ;
            next_state_ = State.MoveNote ;
        }
        else if (dest == NoteDestination.Amp) {
            gotoPosition(TrampConstants.Elevator.Positions.kAmp, Double.NaN, Double.NaN, 
                         TrampConstants.Arm.Positions.kAmp, Double.NaN, Double.NaN);
            next_state_ = State.HoldingAmpPosition ;                         
        }
    }

    public void eject() {
        io_.setManipulatorVoltage(TrampConstants.Manipulator.kEjectVoltage);
        eject_timer_.start();
        state_ = State.Eject ;
    }

    public void transfer() {
        if (state_ == State.HoldingTransferPosition) {
            io_.setManipulatorVoltage(TrampConstants.Manipulator.kTransferVoltage);
            state_ = State.Transferring ;
        }
    }

    public void turtle() {
        gotoPosition(TrampConstants.Elevator.Positions.kStowed, Double.NaN, Double.NaN, 
                     TrampConstants.Arm.Positions.kStowed, Double.NaN, Double.NaN);
        next_state_ = State.Idle ;
    }

    public void stopTransfer() {
        if (state_ == State.Transferring) {
            has_note_ = true ;

            //
            // Switch to a holding PID
            //
            io_.setManipulatorTargetPosition(inputs_.manipulatorPosition);

            state_ = State.Idle ;
        }
    }

    @Override
    public void simulationPeriodic() {
        io_.simulate(getRobot().getPeriod()) ;
    }
    
    @Override
    public void periodic() {
        io_.updateInputs(inputs_) ;
        Logger.processInputs("tramp", inputs_);

        if (climber_dir_ == ClimberDir.Up) {
            if (inputs_.climberPosition >= climber_target_) {
                io_.setClimberMotorVoltage(0.0);
                climber_dir_ = ClimberDir.None ;
            }
        }
        else if (climber_dir_ == ClimberDir.Down) {
            if (inputs_.climberPosition <= climber_target_) {
                io_.setClimberMotorVoltage(0.0);
                climber_dir_ = ClimberDir.None ;
            }
        }

        switch(state_) {
            case Idle:
                break;

            case Eject:
                if (eject_timer_.isExpired()) {
                    io_.setManipulatorVoltage(0.0);
                    has_note_ = false ;
                    state_ = State.Idle ;
                }
                break;

            case GotoDirectToTarget:
                if (isElevatorReady() && isArmReady()) {
                    state_ = next_state_ ;
                }
                break ;

            case CrossMinToMax:
                if (isElevatorReady()) {
                    //
                    // The elevator is at the keepout height, move the arm to the target position
                    //
                    setArmPosition(end_target_arm_);
                    state_ = State.CrossMinToMaxWaitArm ;
                }
                break ;

            case CrossMinToMaxWaitArm:
                if (inputs_.armPosition > TrampConstants.KeepOut.kMaxArm) {
                    //
                    // The arm is clear, go to the final positions
                    //
                    setElevatorPosition(end_target_elev_);
                    setArmPosition(end_target_arm_);
                    state_ = State.GotoDirectToTarget ;
                }
                break ;

            case CrossMaxToMin:
                if (isElevatorReady()) {
                    //
                    // The elevator is at the keepout height, move the arm to the target position
                    //
                    setArmPosition(end_target_arm_);
                    state_ = State.CrossMaxToMinWaitArm ;
                }
                break ;

            case CrossMaxToMinWaitArm:
                if (inputs_.armPosition < TrampConstants.KeepOut.kMinArm) {
                    //
                    // The arm is clear, go to the final positions
                    //
                    setElevatorPosition(end_target_elev_);
                    setArmPosition(end_target_arm_);
                    state_ = State.GotoDirectToTarget ;
                }
                break ;

            case HoldingTransferPosition:
                break ;

            case HoldingAmpPosition:
                break ;

            case MoveNote:
                manipulator_target_ = inputs_.manipulatorPosition + TrampConstants.Manipulator.kTrapMoveNoteDistance ;
                io_.setManipulatorVoltage(TrampConstants.Manipulator.kTrapMoveNotePower);
                state_ = State.MovingNote ;
                break ;

            case MovingNote:                
                if (inputs_.manipulatorPosition >= manipulator_target_) {
                    io_.setManipulatorVoltage(0.0);
                    state_ = State.HoldingTrapPosition ;
                }
                break ;

            case HoldingTrapPosition:
                break ;

            case Transferring:
                break ;

            case Shooting:
                if (shoot_timer_.isExpired()) {
                    io_.setManipulatorVoltage(0.0);
                    has_note_ = false ;
                    gotoPosition(TrampConstants.Elevator.Positions.kStowed, Double.NaN, Double.NaN,
                                 TrampConstants.Arm.Positions.kStowed, Double.NaN, Double.NaN) ;
                    next_state_ = State.Idle ;
                }
                break ;

            case Climbing:
                if (climber_dir_ == ClimberDir.None) {
                    gotoPosition(TrampConstants.Elevator.Positions.kTrap1, Double.NaN, Double.NaN,
                                 TrampConstants.Arm.Positions.kTrap1, Double.NaN, Double.NaN) ;
                    next_state_ = State.Trap1 ;
                }
                break ;

            case Trap1:
                gotoPosition(TrampConstants.Elevator.Positions.kTrap2, Double.NaN, Double.NaN,
                                TrampConstants.Arm.Positions.kTrap2, Double.NaN, Double.NaN) ;
                next_state_ = State.StartDepositNote ;
                break ;

            case StartDepositNote:
                //
                // Ok, trap 2 is done, need to deposit the note
                //
                io_.setManipulatorVoltage(TrampConstants.Manipulator.kDepositPower);
                deposit_trap_timer_.start();
                state_ = State.DepositingNote ;
                break ;

            case DepositingNote:
                if (deposit_trap_timer_.isExpired()) {
                    gotoPosition(TrampConstants.Elevator.Positions.kTrap3, Double.NaN, Double.NaN,
                                TrampConstants.Arm.Positions.kTrap3, Double.NaN, Double.NaN) ;
                    next_state_ = State.Trap3 ;                    
                }
                break ;

            case Trap3:
                gotoPosition(TrampConstants.Elevator.Positions.kTrap4, Double.NaN, Double.NaN,
                             TrampConstants.Arm.Positions.kTrap4, Double.NaN, Double.NaN) ;
                next_state_ = State.Idle ;
                break ;

            case BasicClimbMovingElevatorArm:
                climberUp() ;
                state_ = State.BasicClimbMovingClimber ;
                break ;

            case BasicClimbMovingClimber:
                if (climber_dir_ == ClimberDir.None) {
                    state_ = State.BasicClimbReady ;
                }
                break ;

            case BasicClimbReady:
                break ;

            case BasicClimbDown:
                if (climber_dir_ == ClimberDir.None) {
                    state_ = State.Idle ;
                }
                break ;
        }

        String aux = "" ;
        if (state_ == State.GotoDirectToTarget) {
            aux = ":" + target_arm_ + ":" + target_elev_ ;
        }
        
        if (climber_dir_ != ClimberDir.None) {
            aux += ":" + climber_dir_.toString() ;
        }
        
        Logger.recordOutput("tramp-state", state_ + aux);
        Logger.recordOutput("elev-target", target_elev_);
        Logger.recordOutput("arm-target", target_arm_);
        Logger.recordOutput("manipulator-voltage", io_.getManipulatorVoltage());
        Logger.recordOutput("climber-voltage", io_.getClimberMotorVoltage());
        Logger.recordOutput("is-elev-ready", isElevatorReady());
        Logger.recordOutput("is-arm-ready", isArmReady());
    }

    private void climberUp() {
        io_.setClimberMotorVoltage(TrampConstants.Climber.kMoveClimberVoltage);
        climber_target_ = TrampConstants.Climber.kClimberUpPosition ;
        climber_dir_ = ClimberDir.Up ;
    }

    private void climberDown() {
        io_.setClimberMotorVoltage(-TrampConstants.Climber.kMoveClimberVoltage);
        climber_target_ = TrampConstants.Climber.kClimberDownPosition ;
        climber_dir_ = ClimberDir.Down;
    }

    private boolean isElevatorReady() {
        return 
            Math.abs(inputs_.elevatorPosition - target_elev_) < elevpostol_ &&
            Math.abs(inputs_.elevatorVelocity) < elevveltol_ ;
    }   
    
    private boolean isArmReady() {
        return 
            Math.abs(inputs_.armPosition - target_arm_) < armpostol_ &&
            Math.abs(inputs_.armVelocity) < armveltol_ ;
    }     

    private void setElevatorPosition(double pos) {
        target_elev_ = pos ;
        io_.setElevatorTargetPos(pos);
    }

    private void setArmPosition(double pos) {
        target_arm_ = pos ;
        io_.setArmTargetPos(pos);
    }

    private void gotoPosition(double elevpos, double elevpostol, double elevveltol, double armpos, double armpostol, double armveltol) {
        if (Double.isNaN(elevpostol))
            elevpostol_ = TrampConstants.Elevator.kTargetPosTolerance ;
        else
            elevpostol_ = elevpostol ;

        if (Double.isNaN(elevveltol))
            elevveltol_ = TrampConstants.Elevator.kTargetVelTolerance ;
        else
            elevveltol_ = elevveltol ;
            
        if (Double.isNaN(armpostol))
            armpostol_ = TrampConstants.Arm.kTargetPosTolerance ;
        else
            armpostol_ = armpostol ;

        if (Double.isNaN(armveltol))
            armveltol_ = TrampConstants.Arm.kTargetVelTolerance ;
        else
            armveltol_ = armveltol ;

        end_target_arm_ = armpos ;
        end_target_elev_ = elevpos ;

        if (inputs_.armPosition < TrampConstants.KeepOut.kMinArm && armpos > TrampConstants.KeepOut.kMaxArm) {
            //
            // We are crossing the keepout region from min to max
            //
            if (elevpos > TrampConstants.KeepOut.kElevatorHeight) {
                //
                // The eventual elevator height is above the keepout height, so just go there
                //
                setElevatorPosition(elevpos) ;
            }
            else {
                //
                // The eventual elevator height is below the keepout height, so go to the keepout height first
                //
                setElevatorPosition(TrampConstants.KeepOut.kElevatorHeight) ;
            }

            //
            // And move the ARM so it is staged at the boundary of the keepout region
            // and ready to move as soon as the elevator is above the keepout height
            //
            setArmPosition(TrampConstants.KeepOut.kMinArm);

            state_ = State.CrossMinToMax ;
        }
        else if (inputs_.armPosition > TrampConstants.KeepOut.kMaxArm && armpos < TrampConstants.KeepOut.kMinArm) {
            //
            // We are crossing the keepout region from max to min
            //
            if (elevpos > TrampConstants.KeepOut.kElevatorHeight) {
                //
                // The eventual elevator height is above the keepout height, so just go there
                //
                setElevatorPosition(elevpos) ;
            }
            else {
                //
                // The eventual elevator height is below the keepout height, so go to the keepout height first
                //
                setElevatorPosition(TrampConstants.KeepOut.kElevatorHeight) ;
            }

            //
            // And move the ARM so it is staged at the boundary of the keepout region
            // and ready to move as soon as the elevator is above the keepout height
            //
            setArmPosition(TrampConstants.KeepOut.kMaxArm);

            state_ = State.CrossMaxToMin ;            
        }
        else {
            //
            // There is no interference, go straight to the target position
            //
            setArmPosition(armpos) ;
            setElevatorPosition(elevpos);
            state_ = State.GotoDirectToTarget ;
        }
    }

    private SysIdRoutine elevatorSysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(3) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setElevatorMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logElevatorMotor(log),
                                        this) ;

        return new SysIdRoutine(cfg, mfg) ;
    }

    private SysIdRoutine armSysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(3) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setArmMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logArmMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }   
    
    private SysIdRoutine climberSysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(3) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setClimberMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logClimberMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    } 

    private SysIdRoutine manipulatorSysIdRoutine() {
        Measure<Voltage> step = Units.Volts.of(3) ;
        Measure<Time> to = Units.Seconds.of(10) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setManipulatorVoltage(volts.magnitude()),
                                        (log) -> io_.logManipulatorMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }     

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return elevatorSysIdRoutine().quasistatic(dir) ;
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction dir) {
        return elevatorSysIdRoutine().dynamic(dir) ;
    }   

    public Command manipulatorSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return manipulatorSysIdRoutine().quasistatic(dir) ;
    }

    public Command manipulatorSysIdDynamic(SysIdRoutine.Direction dir) {
        return manipulatorSysIdRoutine().dynamic(dir) ;
    }       
    
    public Command armSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return armSysIdRoutine().quasistatic(dir) ;
    }

    public Command armSysIdDynamic(SysIdRoutine.Direction dir) {
        return armSysIdRoutine().dynamic(dir) ;
    } 
    
    public Command climberSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return climberSysIdRoutine().quasistatic(dir) ;
    }

    public Command climberSysIdDynamic(SysIdRoutine.Direction dir) {
        return climberSysIdRoutine().dynamic(dir) ;
    }     

    public List<TalonFX> getCTREMotors() {
        return io_.getCTREMotors() ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return io_.getRevRoboticsMotors() ;
    }    
}
