package org.xero1425;

import edu.wpi.first.wpilibj2.command.Command;

public class XeroAutoCommand extends Command {
    private XeroRobot robot_ ;

    public XeroAutoCommand(XeroRobot robot) {
        robot_ = robot ;
    }

    public XeroRobot getRobot() {
        return robot_ ;
    }

    @Override
    public void execute() {
    }
}
