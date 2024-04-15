package frc.robot.subsystems.tramp;

public class TrampConstants {
    public class KeepOut {
        public static final double kElevatorHeight = 0.21 ;
        public static final double kMinArm = 5.0 ;
        public static final double kMaxArm = 75.0 ;
    }

    public class Elevator {
        public static final int kMotorId = 6 ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kMetersPerRev = 0.003551136 ;
        public static final double kTargetPosTolerance = 0.5 ;
        public static final double kTargetVelTolerance = 1.0 ;

        public class Positions {
            public static final double kStowed = 0.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kAmp = 0.2 ;
            public static final double kTrap = 0.2411 ;
        }

        public class PID {
            public static final double kP = 1000.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.0 ;
            public static final double kA = 1000.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        };        
    }

    public class Arm {
        public static final int kMotorId = 7 ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kDegreesPerRev = 24.30790007 ;
        public static final double kTargetPosTolerance = 0.5 ;
        public static final double kTargetVelTolerance = 1.0 ;

        public class Positions {
            public static final double kStowed = 0.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kAmp = 225.0 ;
            public static final double kTrap = 180 ;            
        }     
        
        public class PID {
            public static final double kP = 100.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.0 ;
            public static final double kA = 100.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        };        
    }

    public class Manipulator {
        public static final int kMotorId = 9 ;
        public static final double kCurrentLimit = 60.0 ;        
        public static final double kEjectVoltage = 6.0 ;
        public static final double kEjectTime = 0.5 ;
        public static final double kTransferVoltage = 1.2 ;
    }
}
