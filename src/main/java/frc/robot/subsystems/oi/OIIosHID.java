package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.oi.OISubsystem.LEDState;

public class OIIosHID implements OIIos {
    private static final int kMaxLeds = 8 ;
    private static final int kFastLoopCount = 10 ;
    private static final int kSlowLoopCount = 25 ;
        
    private GenericHID hid_ ;    

    private boolean led_onoff_[];
    private OISubsystem.LEDState led_state_[] ;

    private boolean fast_on_off_ ;
    private int fast_on_off_loops_ ;
    private boolean slow_on_off_ ;
    private int slow_on_off_loops_ ;    

    public OIIosHID(int port) {
        hid_ = new GenericHID(port) ;

        led_onoff_ = new boolean[kMaxLeds] ;
        led_state_ = new OISubsystem.LEDState[kMaxLeds] ;
        for(int i = 0 ; i < kMaxLeds ; i++) {
            led_state_[i] = LEDState.Off ;
        }

        fast_on_off_ = false ;
        fast_on_off_loops_ = 0 ;
        slow_on_off_ = false ;
        slow_on_off_loops_ = 0 ;        
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

        updateLEDs() ;
    }

    @Override
    public void setLED(int index, LEDState st) {
        led_state_[index - 1] = st ;        
    }

    public void updateLEDs() {
        fast_on_off_loops_++ ;
        if (fast_on_off_loops_ == kFastLoopCount) {
            fast_on_off_ = !fast_on_off_ ;
            fast_on_off_loops_ = 0 ;
        }

        slow_on_off_loops_++ ;
        if (slow_on_off_loops_ == kSlowLoopCount) {
            slow_on_off_ = !slow_on_off_ ;
            slow_on_off_loops_ = 0 ;
        }

        for(int i = 1 ; i <= kMaxLeds ; i++) {
            boolean desired = false ;

            switch(led_state_[i - 1]) {
                case On:
                    desired = true ;
                    break ;
                case Off:
                    desired = false ;
                    break ;
                case Slow:
                    desired = slow_on_off_ ;
                    break ;
                case Fast:
                    desired = fast_on_off_ ;
                    break ;
            }

            if (desired != led_onoff_[i - 1]) {
                hid_.setOutput(i, desired);
                led_onoff_[i - 1] = desired ;
            }
        }
    }
}
