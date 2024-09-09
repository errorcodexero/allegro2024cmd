package org.xero1425.base;

import edu.wpi.first.wpilibj2.command.Command;

public class XeroAutoCommand extends Command {

    private XeroRobot robot_;
    private String name_;
    private String desc_;

    public XeroAutoCommand(XeroRobot robot, String name, String desc) {
        robot_ = robot;
        name_ = name;
        desc_ = desc;

        setName(name) ;
    }

    public String toString() {
        return name_;
    }

    public String getDescription() {
        return desc_;
    }

    public XeroRobot getRobot() {
        return robot_;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }
}
