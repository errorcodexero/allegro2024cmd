package frc.robot.constants;

public class RobotConstants {
    public static final Mode currentMode = Mode.SIM ;

    public static enum Mode
    {
        REAL,
        SIM,
        REPLAY
    }

    public static final class PathFollowing {
        public static final double kXYTolerance = 0.05 ;
        public static final double kAngleTolerance = 2.0 ;
        public static final double kMaxVelocity = 60.0 ;
        public static final double kMaxAccel = 60.0 ;

        public static final class XCtrl {
            public static final double kP = 2.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
        }
        public static final class YCtrl {
            public static final double kP = 2.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
        }        
        public static final class RotCtrl {
            public static final double kP = 14.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
        }        
    }
}
