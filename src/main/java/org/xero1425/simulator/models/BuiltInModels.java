package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.ModelFactory;

public class BuiltInModels {
    private BuiltInModels() {
    }

    static public void registerBuiltinModels(ModelFactory factory) {
        factory.registerModel("fms", "org.xero1425.simulator.models.FMSModel");
        factory.registerModel("drivergamepad", "org.xero1425.simulator.models.DriverGamepadModel");
        factory.registerModel("swervedrive", "org.xero1425.simulator.models.SwerveDriveModel");
    }
}
