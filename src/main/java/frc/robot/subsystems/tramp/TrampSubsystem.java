package frc.robot.subsystems.tramp;

import org.littletonrobotics.junction.Logger;
import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;
import org.xero1425.XeroTimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.NoteDestination;

public class TrampSubsystem extends XeroSubsystem {
    private enum State {
        Idle,
        Eject,
        GotoDirectToTarget,
        CrossMinToMax,
        CrossMinToMaxWaitArm,
        CrossMaxToMin,
        CrossMaxToMinWaitArm,
        HoldingTransferPosition,
        HoldingDestinationPosition,
        Transferring
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

    private NoteDestination destination_ ;

    private XeroTimer eject_timer_ ;

    private TrampEjectCommand eject_command_ ;
    private TrampTurtleCommand turtle_command_ ;

    public TrampSubsystem(XeroRobot robot) throws Exception {
        super(robot, "trap-arm") ;

        io_ = new TrampIOHardware() ;
        inputs_ = new TrampIOInputsAutoLogged() ;

        eject_timer_ = new XeroTimer(robot, "tramp-eject", TrampConstants.Manipulator.kEjectTime) ;

        eject_command_ = new TrampEjectCommand(this) ;
        state_ = State.Idle ;

        destination_ = NoteDestination.Speaker ;        
    }

    public boolean isIdle() {
        return state_ == State.Idle ;
    }

    public boolean isInTransferPosition() {
        return state_ == State.HoldingTransferPosition ;
    }

    public boolean isInDestinationPosition() {
        return state_ == State.HoldingDestinationPosition ;
    }    

    public Command ejectCommand() {
        return eject_command_ ;
    }

    public Command turtleCommand() {
        return turtle_command_ ;
    }

    public Command targetSpeakerCommand() {
        return runOnce(() -> destination_ = NoteDestination.Speaker) ;
    }

    public Command targetTrapCommand() {
        return runOnce(() -> destination_ = NoteDestination.Trap) ;
    }    

    public Command targetAmpCommand() {
        return runOnce(() -> destination_ = NoteDestination.Amp) ;
    }    

    public void moveToTransferPosition() {
        gotoPosition(TrampConstants.Elevator.Positions.kTransfer, Double.NaN, Double.NaN, TrampConstants.Arm.Positions.kTransfer, Double.NaN, Double.NaN);
        next_state_ = State.HoldingTransferPosition ;
    }

    public void moveToDestinationPosition() {
        if (destination_ == NoteDestination.Trap) {
            gotoPosition(TrampConstants.Elevator.Positions.kTrap, Double.NaN, Double.NaN, 
                         TrampConstants.Arm.Positions.kTrap, Double.NaN, Double.NaN);
        }
        else if (destination_ == NoteDestination.Amp) {
            gotoPosition(TrampConstants.Elevator.Positions.kAmp, Double.NaN, Double.NaN, 
                         TrampConstants.Arm.Positions.kAmp, Double.NaN, Double.NaN);
        }

        next_state_ = State.HoldingDestinationPosition ;
    }

    public void eject() {
        io_.setManipulatorVoltage(TrampConstants.Manipulator.kEjectVoltage);
        eject_timer_.start();
        state_ = State.Eject ;
    }

    public void transfer() {
        if (state_ == State.HoldingTransferPosition) {
            io_.setManipulatorVoltage(0.0);
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
            io_.setManipulatorVoltage(0.0);
            state_ = State.Idle ;
        }
    }
    
    @Override
    public void periodic() {
        io_.updateInputs(inputs_) ;
        Logger.processInputs("tramp", inputs_);

        switch(state_) {
            case Idle:
                break;

            case Eject:
                if (eject_timer_.isExpired()) {
                    io_.setManipulatorVoltage(0.0);
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

            case HoldingDestinationPosition:
                break ;

            case Transferring:
                break ;
        }
        
        Logger.recordOutput("tramp-state", state_);
        Logger.recordOutput("elev-target", target_elev_);
        Logger.recordOutput("arm-target", target_arm_);
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
        if (elevpostol == Double.NaN)
            elevpostol_ = TrampConstants.Elevator.kTargetPosTolerance ;
        else
            elevpostol_ = elevpostol ;

        if (elevveltol == Double.NaN)    
            elevveltol_ = TrampConstants.Elevator.kTargetVelTolerance ;
        else
            elevveltol_ = elevveltol ;
            
        if (armpostol == Double.NaN)
            armpostol_ = TrampConstants.Arm.kTargetPosTolerance ;
        else
            armpostol_ = armpostol ;

        if (armveltol == Double.NaN)
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
}
