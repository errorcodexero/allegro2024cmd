package frc.robot.automodes.competition;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AllegroContainer;

public class JustShootCommand extends XeroAutoCommand {

    private AllegroContainer container_ ;
    private boolean shooting_ ;

    private final static String desc = "This auto mode just shoots a note and stays in place" ;
     
    public JustShootCommand(XeroRobot robot, AllegroContainer container) {
        super(robot, "just-shoot", desc) ;

        container_ = container ;
        addRequirements(container.getDriveTrain());
    }

    @Override
    public void setAlliance(Alliance a) {
    }    

    @Override
    public void initialize() {
        container_.getIntakeShooter().setHasNote(true) ;
        container_.getIntakeShooter().manualShoot(
                ThreeNoteConstants.kLowManualUpDown, ThreeNoteConstants.kLowManualUpDownPosTol, ThreeNoteConstants.kLowManualUpDownVelTol, 
                ThreeNoteConstants.kLowManualTilt, ThreeNoteConstants.kLowManualTiltPosTol, ThreeNoteConstants.kLowManualTiltVelTol,
                ThreeNoteConstants.kLowManualShooter, ThreeNoteConstants.kLowManualShooterVelTol,
                true, true) ;
        shooting_ = true ;
    }

    @Override
    public void execute() {
        super.execute() ;

        if (shooting_ == true && !container_.getIntakeShooter().hasNote()) {
            shooting_ = false ;
        }
    }    

    @Override
    public boolean isFinished() {
        return !shooting_ ;
    }

    @Override
    public void end(boolean interrupted) {
        container_.getDriveTrain().stopPath();
    }
}
