package org.xero1425;

import edu.wpi.first.wpilibj2.command.Command;

public class XeroAutoMode {
    private String name_ ;
    private Command command_ ;

    public XeroAutoMode(String name, Command cmd) {
        name_ = name ;
        command_ = cmd ;
    }

    public Command getCommand() {
        return command_ ;
    }

    public String toString() {
        return name_ ;
    }
}
