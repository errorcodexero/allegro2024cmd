package frc.robot.autos;

import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;

import frc.robot.AllegroContainer;

public class AllegroAutoCommand extends XeroAutoCommand {
    private AllegroContainer container_ ;

    public AllegroAutoCommand(XeroRobot robot, String name, AllegroContainer container) {
        super(robot, name) ;
        container_ = container ;
    }

    protected AllegroContainer getContainer() {
        return container_ ;
    }

    @Override
    public void execute() {
        super.execute();
    }
}
