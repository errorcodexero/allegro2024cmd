package frc.robot.subsystems.oi;

import org.littletonrobotics.junction.AutoLog;

public interface OIIos {
    @AutoLog
    public static class OIIosInputs {
        public boolean target1 = false ;
        public boolean target2 = false ;
        public boolean climbUpPrep = false ;
        public boolean climbUpExec = false ;
        public boolean autoTrap = false ;
        public boolean shoot = false ;
        public boolean turtle = false ;
        public boolean abort = false ;
        public boolean eject = false ;
        public boolean collect = false ;
        public boolean manual1 = false ;
        public boolean manual2 = false ;
    }

    public default void updateInputs(OIIosInputs inputs) {
    }

    public default void setLED(int index, boolean on) {
    }
}
