package org.xero1425.base;

import java.util.Map;

import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;

public interface ISubsystemSim {
    SettingsValue getProperty(String name) ;
    Map<String, TalonFX> getCTREMotors() ;
    void startPeriodic() ;
    void endPeriodic() ;
}
