package frc.robot.subsystems.oi;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NoteDestination;
import frc.robot.ShootType;

public class OISubsystem extends XeroSubsystem {
    private NoteDestination note_dest_;
    private ShootType shoot_type_ ;

    private Trigger eject_trigger_ ;
    private Trigger abort_trigger_ ;
    private Trigger turtle_trigger_ ;
    private Trigger shoot_trigger_ ;
    private Trigger collect_trigger_ ;
    private Trigger climb_up_prep_trigger_ ;
    private Trigger climb_up_exec_trigger_ ;
    private Trigger auto_trap_trigger_ ;

    private NoteDestination auto_node_dest_ ;

    private OIIos ios_ ;
    private OIIosInputsAutoLogged inputs_ ;

    public OISubsystem(XeroRobot robot, int port) {
        super(robot, "oi");

        ios_ = new OIIosHID(port) ;
        inputs_ = new OIIosInputsAutoLogged() ;

        auto_node_dest_ = NoteDestination.AutoSpeaker ;
        note_dest_ = NoteDestination.AutoSpeaker ;

        eject_trigger_ = new Trigger(() -> inputs_.eject) ;
        abort_trigger_ = new Trigger(() -> inputs_.abort) ;
        turtle_trigger_ = new Trigger(() -> inputs_.turtle) ;
        shoot_trigger_ = new Trigger(() -> inputs_.shoot) ; 
        collect_trigger_ = new Trigger(() -> inputs_.collect) ;
        climb_up_prep_trigger_ = new Trigger(() -> inputs_.climbUpPrep) ;
        climb_up_exec_trigger_ = new Trigger(() -> inputs_.climbUpExec) ;
        auto_trap_trigger_ = new Trigger(() -> inputs_.autoTrap) ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }    

    public NoteDestination getNoteDestination() {
        return note_dest_ ;
    }

    public Trigger shoot() {
        return shoot_trigger_ ;
    }    

    public Trigger eject() {
        return eject_trigger_ ;
    }

    public Trigger abort() {
        return abort_trigger_ ;
    }

    public Trigger turtle() {
        return turtle_trigger_ ;
    }

    public Trigger collect() {
        return collect_trigger_ ;
    }

    public Trigger climbUpPrep() {
        return climb_up_prep_trigger_ ;
    }

    public Trigger climbUpExec() {
        return climb_up_exec_trigger_ ;
    }

    public Trigger autoTrap() {
        return auto_trap_trigger_ ;
    }

    public void setAutoNoteDestination(NoteDestination dest) {
        auto_node_dest_ = dest ;
    }

    @Override
    public void periodic() {
        periodicStart();

        ios_.updateInputs(inputs_) ;
        Logger.processInputs("oi", inputs_);

        if (DriverStation.isAutonomousEnabled()) {
            note_dest_ = auto_node_dest_ ;
            shoot_type_ = ShootType.Subwoofer ;
        }
        else {
            note_dest_ = mapNoteDestination(inputs_.target1, inputs_.target2) ;
            shoot_type_ = mapShootType(inputs_.manual1, inputs_.manual2) ;
        }

        Logger.recordOutput("oi-note-dest", note_dest_) ;
        Logger.recordOutput("oi-shoot-type", shoot_type_) ;

        periodicEnd();
    }

    private NoteDestination mapNoteDestination(boolean b1, boolean b2)
    {
        NoteDestination dest = NoteDestination.Undefined ;
        
        if (!b1 && !b2) {
            dest = NoteDestination.Amp ;
        } else if (b1 && !b2) {
            dest = NoteDestination.AutoSpeaker ;
        } else if (!b1 && b2) {
            dest = NoteDestination.Trap ;
        }

        return dest ;
    }

    private ShootType mapShootType(boolean b1, boolean b2)
    {
        ShootType dest = ShootType.Undefined ;
        
        if (!b1 && !b2) {
            dest = ShootType.Auto ;
        } else if (b1 && !b2) {
            dest = ShootType.Podium ;
        } else if (!b1 && b2) {
            dest = ShootType.Subwoofer ;
        }

        return dest ;
    }    

    public List<TalonFX> getCTREMotors() {
        return null ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }    
}
