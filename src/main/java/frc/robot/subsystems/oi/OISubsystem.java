package frc.robot.subsystems.oi;

import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NoteDestination;
import frc.robot.ShotType;

public class OISubsystem extends XeroSubsystem {

    private static final String NAME = "oi" ;

    public enum OILed {
        DBReady(1),
        ShooterReady(2),
        TiltReady(3),
        TrackerReady(4),
        ClimbUpPrepEnabled(5),
        ClimbUpExecEnabled(6),
        AutoTrapExecEnabled(8);

        public final Integer value ;

        private OILed(int value) {
            this.value = value ;
        }
    }

    private NoteDestination note_dest_;
    private ShotType shot_type_ ;

    private Trigger eject_trigger_ ;
    private Trigger abort_trigger_ ;
    private Trigger turtle_trigger_ ;
    private Trigger shoot_trigger_ ;
    private Trigger collect_trigger_ ;
    private Trigger climb_up_prep_trigger_ ;
    private Trigger climb_up_exec_trigger_ ;
    private Trigger auto_trap_trigger_ ;

    private OIIos ios_ ;
    private OIIosInputsAutoLogged inputs_ ;

    public OISubsystem(XeroRobot robot, int port) {
        super(robot, NAME);

        ios_ = new OIIosHID(port) ;
        inputs_ = new OIIosInputsAutoLogged() ;

        note_dest_ = NoteDestination.Speaker ;

        eject_trigger_ = new Trigger(() -> inputs_.eject) ;
        abort_trigger_ = new Trigger(() -> inputs_.abort) ;
        turtle_trigger_ = new Trigger(() -> inputs_.turtle) ;
        shoot_trigger_ = new Trigger(() -> inputs_.shoot) ; 
        collect_trigger_ = new Trigger(() -> inputs_.collect) ;
        climb_up_prep_trigger_ = new Trigger(() -> inputs_.climbUpPrep) ;
        climb_up_exec_trigger_ = new Trigger(() -> inputs_.climbUpExec) ;
        auto_trap_trigger_ = new Trigger(() -> inputs_.autoTrap) ;
    }

    public boolean isAbortPressed() {
        return inputs_.abort ;
    }

    public boolean isAutoTrapPressed() {
        return inputs_.autoTrap ;
    }

    public void setLEDState(OILed led, boolean b) {
        ios_.setLED(led.value, b) ;
    }

    public String getPressedString() {
        String str = "" ;

        if (inputs_.eject) {
            if (str.length() > 0)
                str += "," ;
            str += "eject" ;
        }

        if (inputs_.abort) {
            if (str.length() > 0)
                str += "," ;
            str += "abort" ;
        }
        
        if (inputs_.turtle) {
            if (str.length() > 0)
                str += "," ;
            str += "turtle" ;
        }
        
        if (inputs_.shoot) {
            if (str.length() > 0)
                str += "," ;
            str += "shoot" ;
        }
        
        if (inputs_.collect) {
            if (str.length() > 0)
                str += "," ;
            str += "collect" ;
        }
        
        if (inputs_.climbUpPrep) {
            if (str.length() > 0)
                str += "," ;
            str += "climbprep" ;
        }
        
        if (inputs_.climbUpExec) {
            if (str.length() > 0)
                str += "," ;
            str += "climbexec" ;
        }
        
        if (inputs_.autoTrap) {
            if (str.length() > 0)
                str += "," ;
            str += "autotrap" ;
        }        

        return str ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }    

    public NoteDestination getNoteDestination() {
        return note_dest_ ;
    }

    public ShotType getShotType() {
        return shot_type_ ;
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

    @Override
    public void periodic() {
        startPeriodic();

        ios_.updateInputs(inputs_) ;
        Logger.processInputs("oi", inputs_);

        note_dest_ = mapNoteDestination(inputs_.target1, inputs_.target2) ;
        shot_type_ = mapShotType(inputs_.manual1, inputs_.manual2) ;

        if (getVerbose()) {
            Logger.recordOutput("oi:note-dest", note_dest_) ;
            Logger.recordOutput("oi:shoot-type", shot_type_) ;
        }

        endPeriodic();
    }

    private NoteDestination mapNoteDestination(boolean b1, boolean b2)
    {
        NoteDestination dest = NoteDestination.Undefined ;
        
        if (!b1 && !b2) {
            dest = NoteDestination.Amp ;
        } else if (b1 && !b2) {
            dest = NoteDestination.Speaker ;
        } else if (!b1 && b2) {
            dest = NoteDestination.Trap ;
        }

        return dest ;
    }

    private ShotType mapShotType(boolean b1, boolean b2)
    {
        ShotType dest = ShotType.Undefined ;
        
        if (!b1 && !b2) {
            dest = ShotType.Auto ;
        } else if (b1 && !b2) {
            dest = ShotType.Podium ;
        } else if (!b1 && b2) {
            dest = ShotType.Subwoofer ;
        }

        return dest ;
    }    

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }    
}
