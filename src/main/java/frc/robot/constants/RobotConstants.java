package frc.robot.constants;

import org.xero1425.XeroRobot.RobotType;

public class RobotConstants {
    public final RobotType robotType = RobotType.COMPETITION_REAL ;

    public static final boolean kTestMode = false ;
    public static final boolean kReplayMode = false ;
    public static final boolean kCharMode = false ;
    public static final class WhichSubsystem {
        public static final boolean kCharDBSubsystem = true ;
        public static final boolean kCharTiltSubsystem = false ;    
        public static final boolean kCharUpDownSubsystem = false ;        
        public static final boolean kCharShooter1Subsystem = false ;
        public static final boolean kCharShooter2Subsystem = false ;
        public static final boolean kCharElevatorSubsystem = false ;
        public static final boolean kCharArmSubsystem = false ;
        public static final boolean kCharClimberSubsystem = false ;    
        public static final boolean kCharManipulatorSubsystem = false ;
    }

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
            public static final double kP = 6.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
        }
        public static final class YCtrl {
            public static final double kP = 6.0 ;
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
