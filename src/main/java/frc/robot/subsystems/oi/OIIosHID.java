package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;

public class OIIosHID implements OIIos {
    private GenericHID hid_ ;    

    public OIIosHID(int port) {
        hid_ = new GenericHID(port) ;
    }

    @Override
    public void updateInputs(OIIosInputs inputs) {
        if (hid_ != null) {
            inputs.target1 = hid_.getRawButton(OIConstants.Buttons.kTarget1) ;
            inputs.target2 = hid_.getRawButton(OIConstants.Buttons.kTarget2) ;
            inputs.climbUpPrep = hid_.getRawButton(OIConstants.Buttons.kClimbUpPrep) ;
            inputs.climbUpExec = hid_.getRawButton(OIConstants.Buttons.kClimbUpExec) ;
            inputs.autoTrap = hid_.getRawButton(OIConstants.Buttons.kAutoTrap) ;
            inputs.shoot = hid_.getRawButton(OIConstants.Buttons.kShoot) ;
            inputs.turtle = hid_.getRawButton(OIConstants.Buttons.kTurtle) ;
            inputs.abort = hid_.getRawButton(OIConstants.Buttons.kAbort) ;
            inputs.eject = hid_.getRawButton(OIConstants.Buttons.kEject) ;
            inputs.collect = hid_.getRawButton(OIConstants.Buttons.kCollect) ;
            inputs.manual1 = hid_.getRawButton(OIConstants.Buttons.kManual1) ;
            inputs.manual2 = hid_.getRawButton(OIConstants.Buttons.kManual2) ;
        }
    }

    @Override
    public void setLED(int index, boolean on) {
        if (hid_ != null) {
            hid_.setOutput(index, on) ;
        }
    }
}
