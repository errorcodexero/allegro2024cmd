package frc.robot.subsystems.intakeshooter;

import org.xero1425.XeroRobot;

public class IntakeShooterConstants {
    public static final double kvfactor = 60.0 ;
    public static final double kafactor = 80.0 ;
    public static final double kjfactor = 500.0;

    public static final double kCollectDelayTime = 0.1 ;
    public static final double kReverseDelayTime = 0.3 ;
    public static final double kTransferFeederToShooterDelay = 0.4 ;


    public class Feeder {
        public static final int kMotorId = 1 ;
        public static final boolean kInvert = true ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kCollectVoltage = 6.0 ;
        public static final double kTransferVoltage = 4.0 ;
        public static final double kShootVoltage = 12.0 ;
        public static final double kShootTime = 0.4 ;
        public static final double kEjectVoltage = 1.0 ;
    }

    public class UpDown {
        public static final int kMotorId = 2 ;
        public static final boolean kInvert = true ;

        public static final double kCurrentLimit = 60.0 ;
        public static final double kTargetPosTolerance = 0.5 ;
        public static final double kTargetVelTolerance = 1.0 ;
        public static final double kDegreesPerRev = 7.2727273 ;
        public static final double kSimGearRatio = 2.0 ;
        public static final double kSimMotorLoad = 0.00001 ;
        public static final double kMaxPosition = 118.0 ;
        public static final double kMinPosition = -10.0 ;

        public class Positions {
            public static final double kStowed = 118.0 ;
            public static final double kStartTracking = 118.0 ;
            public static final double kCollect = -10.0 ;
            public static final double kTransfer = 90.0 ;
            public static final double kShootNominal = 118.0 ;
            public static final double kEject = 118.0 ;
        }            

        public class PID {
            public static final double kP = 10.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.18 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        } ;

        public class MotionMagic {
            public static final double kV = kvfactor ;
            public static final double kA = kafactor ;
            public static final double kJ = kjfactor ;
        }

        public static final double[] kPwlValues = new double[] {
            0.0, 100.0, 
            2.119, 100.0, 
            2.12, 118.0, 
            5.0, 118.0
        } ;
    }

    public class Tilt {
        public static final int kMotorId = 5 ;
        public static final boolean kInvert = true ;            
        public static final double kCurrentLimit = 60.0 ;
        public static final double kTargetPosTolerance = 2.0 ;
        public static final double kTargetVelTolerance = 1.0 ;
        public static final double kAllowedDeviationFromTrack = 100.0 ;
        public static final double kDegreesPerRev = 20 ;
        public static final double kMinPosition = -72.0 ;
        public static final double kMaxPosition = 45.0 ;
        public static final double kSimGearRatio = 18.0 ;        
        public static final double kSimMotorLoad = 0.1;

        public class Positions {
            public static final double kStowed = -72.0 ;
            public static final double kStartTracking = -50.0 ;
            public static final double kCollect = 45.0 ;
            public static final double kTransfer = 0.0 ;

            public static final double kShootNominal = -50.0 ;
            public static final double kEject = -50.0 ;
        }             
        public class TrackingPIDSlot1 {
            public static final double kP = 5.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.35 ;
            public static final double kV = 0.0 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        };

        public class MovementPIDSlot0 {
            public static final double kP = 5.0 ;
            public static final double kI = 0.0 ;
            public static final double kD = 0.35 ;
            public static final double kV = 0.1 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        };        

        public class MotionMagic {
            // public static final double kV = 15 ;
            public static final double kV = kvfactor * 7.2727273 / 20.0 * 0.78 ;
            public static final double kA = kafactor * 7.2727273 / 20.0 * 0.78 ;
            public static final double kJ = kjfactor * 7.2727273 / 20.0 * 0.78 ;
        }

        public class AbsoluteEncoder {
            public static final int kChannel = 0;
            public static final double kRobotMax = 90.0 ;
            public static final double kRobotMin = -90.0 ;
            public static final double kRobotCalibrationValue = -72.0 ;
            public static final double kEncoderMax = 5.0 ;
            public static final double kEncoderMin = 0.0 ;
            public static final double kEncoderCalibrationValue(XeroRobot robot) {
                if (robot.isPracticeBot())
                    return Practice.kEncoderCalibrationValue ;
                else if (robot.isCompetitionBot())
                    return Competition.kEncoderCalibrationValue ;

                return Simulation.kEncoderCalibrationValue ;
            }
            public class Practice {
                public static final double kEncoderCalibrationValue = 0.0 ;
            }
            public class Competition {
                public static final double kEncoderCalibrationValue = 4.283 ;
            }
            public class Simulation {
                public static final double kEncoderCalibrationValue = 0.0 ;
            }            
        }

        public static final double[] kPwlValues = new double[] {
            1.24,  -64, 
            1.52,  -60,
            1.82,  -55,
            2.119, -55,
            2.12,  -74,
            2.43,  -66,
            2.73,  -64,
            3.04,  -62,
            3.34,  -61,
            3.6399,-59,
            3.64,  -59,
            4.0,   -57
        } ;            
    }

    public class Shooter {
        public static final double kVelocityTolerance = 2.5 ;        
        public static final double kEjectForwardTime = 1.0 ;
        public static final double kEjectPauseTime = 0.5 ;
        public static final double kEjectReverseTime = 1.0 ;
        public static final double kEjectVoltage = 8.0 ;

        public static final double kAutoShootVelocityTol = 5.0 ;            

        public static final double kTransferVelocity = 58.0 ;
        public static final double kTransferVelocityTol = 5.0 ;        
        public static final double kTransferTransferLength = 0.6 ;
        public static final double kTransferContLength = 12.0 ;     

        public static final double kSimGearRatio = 0.6 ;      
        public static final double kSimMotorLoad = 0.001 ;

        public static final double kP = 32.0;
        public static final double kI = 0.0 ;
        public static final double kD = 0.0 ;
        public static final double kV = 0.008 ;
        public static final double kA = 0.0 ;
        public static final double kG = 0.0 ;
        public static final double kS = 0.0 ;   
        public static final double[] kPwlValues = new double[] {
            1.24,  65, 
            1.52,  65,
            1.82,  65,
            2.119, 65,
            2.12,  80,
            2.43,  80,
            2.73,  80,
            3.04,  80,
            3.34,  80,
            3.6399,80,
            3.64,  84,
            4.0,   84            
        } ;            
    }

    public class Shooter1 {
        public static final int kMotorId = 3 ;
        public static final boolean kInvert = false ;             
        public static final double kCurrentLimit = 80.0 ;
    }

    public class Shooter2 {
        public static final int kMotorId = 4 ;
        public static final boolean kInvert = false ;             
        public static final double kCurrentLimit = 80.0 ;    
    }

    public class NoteSensor {
        public static final int kChannel = 1 ;
        public static final boolean kInverted = true ;
    }    
}
