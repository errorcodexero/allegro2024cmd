package frc.robot.subsystems.oi;

public class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOIControllerPort = 2 ;
    public class Buttons {
        public static int kTarget1 = 1 ;
        public static int kTarget2 = 2 ;            
        public static int kClimbUpPrep = 3 ;
        public static int kClimbUpExec = 4 ;
        public static int kAutoTrap = 6 ;
        public static int kShoot = 7 ;
        public static int kTurtle = 8 ;
        public static int kAbort = 9 ;
        public static int kEject = 10 ;
        public static int kCollect = 11 ;
        public static int kManual1 = 12 ;
        public static int kManual2 = 13 ;
    }

    public class LEDs {
        public static int kDBReady = 1 ;
        public static int kShooterVelocityReady = 2 ;
        public static int kTiltReady = 3 ;
        public static int kAprilTagReady = 4 ;
        public static int kClimbUpPrepEnabled = 5 ;
        public static int kClimbUpExecEnabled = 6 ;
        public static int kAutoTrapEnabled = 8 ;
    }
}
