package frc.robot.subsystems.oi;

import org.littletonrobotics.junction.Logger;
import org.xero1425.XeroRobot;
import org.xero1425.XeroSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NoteDestination;

public class OISubsystem extends XeroSubsystem {
    private NoteDestination note_dest_;
    private GenericHID hid_ ;

    private Trigger eject_trigger_ ;
    private Trigger abort_trigger_ ;
    private Trigger turtle_trigger_ ;
    private Trigger shoot_trigger_ ;
    private Trigger collect_trigger_ ;

    public OISubsystem(XeroRobot robot, int port) {
        super(robot, "oi");
        
        hid_ = new GenericHID(port) ;
        note_dest_ = NoteDestination.Speaker ;

        eject_trigger_ = new Trigger(() -> hid_.getRawButton(OIConstants.Buttons.kEject)) ;
        abort_trigger_ = new Trigger(() -> hid_.getRawButton(OIConstants.Buttons.kAbort)) ;
        turtle_trigger_ = new Trigger(() -> hid_.getRawButton(OIConstants.Buttons.kTurtle)) ;
        shoot_trigger_ = new Trigger(() -> hid_.getRawButton(OIConstants.Buttons.kShoot)) ;
        collect_trigger_ = new Trigger(() -> hid_.getRawButton(OIConstants.Buttons.kCollect)) ;
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

    @Override
    public void periodic() {
        boolean t1 = hid_.getRawButton(OIConstants.Buttons.kTarget1) ;
        boolean t2 = hid_.getRawButton(OIConstants.Buttons.kTarget2) ;
        note_dest_ = mapNoteDestination(t1, t2) ;

        Logger.recordOutput("button-collect", hid_.getRawButton(OIConstants.Buttons.kCollect)) ;
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
}
