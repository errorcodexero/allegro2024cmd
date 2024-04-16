package frc.robot.autos;

import org.xero1425.XeroAutoMode;
import org.xero1425.XeroRobot;

import frc.robot.AllegroContainer;

public class FourNoteDynamicAuto extends XeroAutoMode {
    public FourNoteDynamicAuto(XeroRobot robot, AllegroContainer container) {
        super("four-note", new FourNoteDynamicCommand(robot, container));
    }
}
