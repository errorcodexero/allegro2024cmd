package frc.robot.subsystems.tramp;

public class TrampConstants {
    public class KeepOut {
        public static final double kElevatorHeight = 0.21 ;
        public static final double kMinArm = 5.0 ;
        public static final double kMaxArm = 75.0 ;
    }

    public class Elevator {
        public static final int kMotorId = 6 ;
        public static final boolean kInverted = true ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kMetersPerRev = 0.003551136 ;
        public static final double kTargetPosTolerance = 0.5 ;
        public static final double kTargetVelTolerance = 1.0 ;
        public static final double kSimGearRatio = 0.6 ;      
        public static final double kSimMotorLoad = 0.00001 ;        

        public class Positions {
            public static final double kStowed = 0.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kAmp = 0.2 ;
            public static final double kTrap = 0.2411 ;
            public static final double kTrap1 = 0.2522 ;
            public static final double kTrap2 = 0.3222 ;
            public static final double kTrap3 = 0.2411 ;
            public static final double kTrap4 = 0.2411 ;
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
        public static final boolean kInverted = true ;        
        public static final double kCurrentLimit = 60.0 ;
        public static final double kDegreesPerRev = 24.30790007 ;
        public static final double kTargetPosTolerance = 0.5 ;
        public static final double kTargetVelTolerance = 1.0 ;
        public static final double kSimGearRatio = 2.0 ;      
        public static final double kSimMotorLoad = 0.00001 ;        

        public class Positions {
            public static final double kStowed = 0.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kAmp = 225.0 ;
            public static final double kTrap = 180 ;
            public static final double kTrap1 = 121 ;
            public static final double kTrap2 = 121 ;
            public static final double kTrap3 = 150 ;
            public static final double kTrap4 = 225 ;            
        }     
        
        public class PID {
            public static final double kP = 50.0 ;
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
        public static final double kShootTime = 1.5 ;
        public static final double kShootPower = 1.0 ;
        public static final double kTrapMoveNoteDistance = 0.7 ;
        public static final double kTrapMoveNotePower = 0.05 ;
        public static final double kDepositTime = 1.5 ;
        public static final double kDepositPower = -1.0 ;
    }

    public class Climber {
        public static final int kMotorId = 10 ;
        public static final boolean kInverted = true ;        
        public static final double kCurrentLimit = 60.0 ;
        public static final double kSimGearRatio = 2.0 ;      
        public static final double kSimMotorLoad = 0.00001 ;  
        public static final double kMoveClimberVoltage = 12.0 ;
        public static final double kClimberUpPosition = 0.5746 ;
        public static final double kClimberDownPosition = 0.0 ;
    }
}
