package frc.robot.autos;

import org.xero1425.XeroAutoMode;
import org.xero1425.XeroRobot;

import frc.robot.AllegroContainer;

public class ThreeNoteDynamicAuto extends XeroAutoMode {
    public ThreeNoteDynamicAuto(XeroRobot robot, AllegroContainer container) {
        super("three-note", new ThreeNoteDynamicCommand(robot, container));
    }
}
