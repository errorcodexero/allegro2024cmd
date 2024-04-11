package org.xero1425;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXFactory {
    private final static int kApplyTries = 5 ;
    private static TalonFXFactory factory_ = new TalonFXFactory() ;

    public static TalonFXFactory getFactory() {
        return factory_ ;
    }

    //
    // Creates a new TalonFX motor controller in brake mode
    //
    public TalonFX createTalonFX(int id, String bus, boolean invert, double limit) throws Exception {
        TalonFX fx = new TalonFX(id, bus) ;
        TalonFXConfiguration config = new TalonFXConfiguration() ;       
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake ;

        if (limit != Double.NaN) {
            config.CurrentLimits.SupplyCurrentLimit = limit ;
            config.CurrentLimits.SupplyCurrentLimitEnable = true ;
        }

        checkError("TalonFXMotorController - apply configuration", () -> fx.getConfigurator().apply(config));        
        return fx ;
    }       

    public TalonFX createTalonFX(int id, String bus, boolean invert) throws Exception {
        return createTalonFX(id, invert, Double.NaN) ;
    }     

    public TalonFX createTalonFX(int id, boolean invert) throws Exception {
        return createTalonFX(id, "", invert, Double.NaN) ;
    }   

    public TalonFX createTalonFX(int id, boolean invert, double limit) throws Exception {
        return createTalonFX(id, "", invert, limit) ;
    }     

    private static void checkError(String msg, Supplier<StatusCode> toApply) throws Exception {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = kApplyTries ;
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            msg = msg + " - " + code.toString() ;
            throw new Exception(msg) ;
        }
    }    
}
