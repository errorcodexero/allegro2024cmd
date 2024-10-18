package frc.robot.subsystems.tramp;

public class TrampConstants {
    public class KeepOut {
        public static final double kElevatorHeight = 0.21 ;
        public static final double kMinArm = 5.0 ;
        public static final double kMaxArm = 75.0 ;
    }

    public class Trap {
        public static final double kDelayTime = 0.5 ;
    }

    public class Elevator {
        public static final int kMotorId = 6 ;
        public static final boolean kInverted = true ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kMetersPerRev = 0.003896166;
        public static final double kTargetPosTolerance = 0.0508 ;
        public static final double kTargetVelTolerance = 1.000 ;
        public static final double kSimGearRatio = 0.6 ;      
        public static final double kSimMotorLoad = 0.00001 ;      
        public static final double kMinPosition = 0.0 ;
        public static final double kMaxPosition = 0.430 ;

        public class Positions {
            public static final double kStowed = 0.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kAmp = 0.2 ;
            public static final double kTrapBeforeClimb = 0.2411 ;
            public static final double kTrapAfterClimb1 = 0.2522 ;
            public static final double kTrapAfterClimb2 = 0.3222 ;
            public static final double kTrapAfterDeposit1 = 0.2411 ;
            public static final double kTrapAfterDeposit2 = 0.2411 ;
            public static final double kBasicClimb = 0.2 ;
        }

        public class Real {
            public class PID {
                public static final double kP = 8.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.0 ;
                public static final double kA = 0.0 ;
                public static final double kG = 0.25 ;
                public static final double kS = 0.0 ;
            };        

            public class MotionMagic {
                public static final double kMaxVelocity = 89.0 ;
                public static final double kMaxAcceleration = 2246 ;
                public static final double kJerk = 10000 ;
            }
        }
    
        public class Simulated {
            public class PID {
                public static final double kP = 1.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.25 ;
                public static final double kA = 0.0 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            };          

            public class MotionMagic {
                public static final double kMaxVelocity = 89.0 ;
                public static final double kMaxAcceleration = 2246 ;
                public static final double kJerk = 10000 ;
            }
        }
    }

    public class Arm {
        public static final int kMotorId = 7 ;
        public static final boolean kInverted = true ;        
        public static final double kCurrentLimit = 60.0 ;
        public static final double kDegreesPerRev = 24.30790007 ;
        public static final double kTargetPosTolerance = 4.0 ;
        public static final double kTargetVelTolerance = 100.0 ;
        public static final double kMinPosition = -12.0 ;
        public static final double kMaxPosition = 300.0 ;

        public class Positions {
            public static final double kStowed = 0.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kAmp = 230.0 ;
            public static final double kTrapBeforeClimb = 180 ;
            public static final double kTrapAfterClimb1 = 139 ;
            public static final double kTrapAfterClimb2 = 139 ;
            public static final double kTrapAfterDeposit1 = 149.6 ;
            public static final double kTrapAfterDeposit2 = 225 ;  
            public static final double kBasicClimb = 180.0 ;          
        }     

        public class Real {        
            public class PID {
                public static final double kP = 4.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.1 ;
                public static final double kA = 0.0;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            };        

            public class MotionMagic {
                public static final double kMaxVelocity = 24.0 ;
                public static final double kMaxAcceleration = 180.0 ;
                public static final double kJerk = 10000 ;
            }
        }

        public class Simulated {
            public class PID {
                public static final double kP = 1.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.25 ;
                public static final double kA = 0.0;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            };        

            public class MotionMagic {
                public static final double kMaxVelocity = 24.0 ;
                public static final double kMaxAcceleration = 180.0 ;
                public static final double kJerk = 10000 ;
            }            
        }
    }

    public class Manipulator {
        public static final int kMotorId = 9 ;
        public static final boolean kInverted = true ;
        public static final double kCurrentLimit = 60.0 ;        
        public static final double kEjectVoltage = -6.0 ;
        public static final double kEjectTime = 0.5 ;
        public static final double kShootTime = 0.8 ;
        public static final double kShootPower = -5.0 ;
        public static final double kTrapMoveNoteDistance = 0.2 ;
        public static final double kTrapMoveNoteVelocity = 5.0 ;
        public static final double kDepositTime = 1.5 ;
        public static final double kDepositVelocity = 10 ;
        public static final double kHoldNoteTime = 0.2 ;
        public static final double kFreeWheelTransferDistance = 0.5 ;

        public static final double kTransferVoltage = 1.2 ;
        public static final double kTransferVelocity = 25.0 ;
        public static final double kTransferTime = 0.2;

        public class ThruBoreEncoder {
            public static int kEncoderA = 2 ;
            public static int kEncoderB = 3 ;
            public static boolean kEncoderInverted = false ;
            public static double kEncoderDistancePerPulse = 1.0 / 2048.0 ;
        }

        public class PositionPID {
            public static final double kP = 10 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.0 ;
            public static final double kA = 0.0;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        };   
        
        public class VelocityPID {
            public static final double kP = 0.01 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.010189;
            public static final double kA = 0.0;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        };         
    }

    public class Climber {
        public static final int kMotorId = 10 ;
        public static final boolean kInverted = false ;        
        public static final double kCurrentLimit = 60.0 ;
        public static final double kSimGearRatio = 2.0 ;      
        public static final double kSimMotorLoad = 0.00001 ;  
        public static final double kMoveClimberVoltage = 12.0 ;
        public static final double kMoveClimberSlowVoltage = 4.0 ;        
        public static final double kClimberUpPosition = 0.5746 ;
        public static final double kClimberDownPosition = 0.0 ;
        public static final double kClimberDownSlowMargin = 0.1 ;
        public static final double kMaxPosition = 0.5746 ;
        public static final double kMinPosition = 0.0 ;
        public static final double kMetersPerRev = 0.00000134301757813 * 2048.0 ;
    }
}
