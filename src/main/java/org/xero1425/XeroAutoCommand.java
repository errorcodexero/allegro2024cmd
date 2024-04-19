package org.xero1425;

import edu.wpi.first.wpilibj2.command.Command;

public class XeroAutoCommand extends Command {
    private XeroRobot robot_ ;
    private String name_ ;

    public XeroAutoCommand(XeroRobot robot, String name) {
        robot_ = robot ;
        name_ = name ;
    }

    public String toString() {
        return name_ ;
    }

    public XeroRobot getRobot() {
        return robot_ ;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }
}
