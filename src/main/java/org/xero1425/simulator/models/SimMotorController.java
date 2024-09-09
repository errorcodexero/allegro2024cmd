package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationEngine;

public abstract class SimMotorController implements ISimMotorController {
    final static boolean kPlotMotorSims = false ;

    private SimulationEngine engine_ ;

    protected SimMotorController(SimulationEngine engine, String bus, int canid) {
        engine_ = engine ;
    }

    public void addPlotData(double bvolts, double mvolts, double pos, double vel) {
    }

    protected SimulationEngine getEngine() {
        return engine_ ;
    }
}
