package frc.models;

import frc.robot.subsystems.oi.OIConstants;
import org.xero1425.simulator.models.OIBaseModel;
import org.xero1425.simulator.engine.SimulationEngine;

import java.util.Map;

public class AllegroOIModel extends OIBaseModel {
    private static final Map<String, Integer> buttonMap = Map.of(
        "eject", OIConstants.Buttons.kEject,
        "abort", OIConstants.Buttons.kAbort,
        "turtle", OIConstants.Buttons.kTurtle,
        "shoot", OIConstants.Buttons.kShoot,
        "collect", OIConstants.Buttons.kCollect,
        "climb-up-prep", OIConstants.Buttons.kClimbUpPrep,
        "climb-up-exec", OIConstants.Buttons.kClimbUpExec,
        "auto-trap", OIConstants.Buttons.kAutoTrap
    ) ;    

    public AllegroOIModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst, buttonMap);

        registerMultiButton("note-target", new int[] { OIConstants.Buttons.kTarget1, OIConstants.Buttons.kTarget2 }, new String[] { "amp", "speaker", "trap" }) ;
        registerMultiButton("shoot-type", new int[] { OIConstants.Buttons.kManual1, OIConstants.Buttons.kManual2 }, new String[] { "auto", "podium", "subwoofer"}) ;
    }
}