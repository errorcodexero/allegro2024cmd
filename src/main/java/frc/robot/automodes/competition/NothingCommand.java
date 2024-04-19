package frc.robot.automodes.competition;

import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;

public class NothingCommand extends XeroAutoCommand {

    private final static String desc = "This auto mode starts anywhere on the field and does nothing." ;   

    public NothingCommand(XeroRobot robot) {
        super(robot, "do-nothing", desc) ;
    }
}
