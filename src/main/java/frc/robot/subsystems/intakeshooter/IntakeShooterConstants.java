package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.XeroRobot;

public class IntakeShooterConstants {
    public static final double kvfactor = 60.0 ;
    public static final double kafactor = 80.0 ;
    public static final double kjfactor = 500.0;

    public static final double kCollectDelayTime = 0.0 ;
    public static final double kReverseDelayTime = 0.3 ;

    public class ManualShotFerry {
        public static final double kUpDownPos = 110.0 ;
        public static final double kUpDownPosTolerance = 5.0 ;
        public static final double kUpDownVelTolerance = 5.0 ;
        public static final double kTiltPos = -70.0 ;
        public static final double kTiltPosTolerance = 4.0 ;
        public static final double kTiltVelTolerance = 16.0 ;
        public static final double kShooterVel = 75.0 ;
        public static final double kShooterVelTolerance = 5.0 ;
    }

    public class ManualShotPodium {
        public static final double kUpDownPos = 118.0 ;
        public static final double kUpDownPosTolerance = 5.0 ;
        public static final double kUpDownVelTolerance = 5.0 ;
        public static final double kTiltPos = -64.0 ;
        public static final double kTiltPosTolerance = 4.0 ;
        public static final double kTiltVelTolerance = 5.0 ;
        public static final double kShooterVel = 75.0 ;
        public static final double kShooterVelTolerance = 5.0 ;
    }

    public class ManualShotSubwoofer {
        public static final double kUpDownPos = 100.0 ;
        public static final double kUpDownPosTolerance = 5.0 ;
        public static final double kUpDownVelTolerance = 5.0 ;
        public static final double kTiltPos = -65.0 ;
        public static final double kTiltPosTolerance = 3.0 ;
        public static final double kTiltVelTolerance = 1e32 ;
        public static final double kShooterVel = 75.0 ;
        public static final double kShooterVelTolerance = 5.0 ;
    }

    public class Feeder {
        public static final int kMotorId = 1 ;
        public static final boolean kInvert = true ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kCollectVoltage = 6.0 ;
        public static final double kTransferVoltage = 3.0 ;
        public static final double kShootVoltage = 12.0 ;
        public static final double kShootTime = 0.4 ;
        public static final double kEjectVoltage = 6.0 ;
    }

    public class UpDown {
        public static final int kMotorId = 2 ;
        public static final boolean kInvert = true ;

        public static final double kCurrentLimit = 60.0 ;
        public static final double kTargetPosTolerance = 5.0 ;
        public static final double kTargetVelTolerance = 5.0 ;
        public static final double kDegreesPerRev = 7.2727273 ;
        public static final double kSimGearRatio = 2.0 ;
        public static final double kSimMotorLoad = 0.00001 ;
        public static final double kMaxPosition = 118.0 ;
        public static final double kMinPosition = -10.0 ;

        public class Positions {
            public static final double kStowed = 118.0 ;
            public static final double kFinishTransfer = 60.0 ;
            public static final double kStartTracking = 118.0 ;
            public static final double kCollect = -6.0 ;
            public static final double kTransfer = 90.0 ;
            public static final double kTransferPosTol = 5.0 ;
            public static final double kTransferVelTol = 5.0 ;                        
            public static final double kShootNominal = 118.0 ;
            public static final double kEject = 118.0 ;
        }

        public class Real {
            public class PID {
                public static final double kP = 40.0 ;
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
        }

        public class Simulated {
            public class PID {
                public static final double kP = 1.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.25;
                public static final double kA = 0.0 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            } ;

            public class MotionMagic {
                public static final double kV = kvfactor ;
                public static final double kA = kafactor ;
                public static final double kJ = kjfactor ;
            }
        }

        public static final double[] kPwlValues = new double[] {
            0.0, 100.0,
            2.319, 100.0,
            2.32, 118.0,
            5.0, 118.0
        } ;
    }

    public class Tilt {
        public static final int kMotorId = 5 ;
        public static final boolean kInvert = true ;
        public static final double kCurrentLimit = 60.0 ;
        public static final double kTargetPosTolerance = 5.0 ;
        public static final double kTargetVelTolerance = 5.0 ;
        public static final double kTargetShootingPosToleranceFarDist = 4.5 ;
        public static final double kTargetShootingPosToleranceNearDist = 1.0 ;
        public static final double kTargetShootingPosToleranceNear = 5.0 ;
        public static final double kTargetShootingPosToleranceFar = 1.0 ;        
        public static final double kTargetShootingVelTolerance = 5.0 ;
        public static final double kAllowedDeviationFromTrack = 100.0 ;
        public static final double kDegreesPerRev = 20 ;
        public static final double kMinPosition = -72.0 ;
        public static final double kMaxPosition = 45.0 ;
        public static final double kSimGearRatio = 18.0 ;
        public static final double kSimMotorLoad = 0.1;

        // The velocity of the tilt must be below this threshold in order to
        // shoot a note.  This is measured using the absolute encoder that is
        // mounting on the tilt mechansim beyond the gearing
        public static final double kMaxAbsoluteTiltVelocity = 4.0 ;

        // The velocity of the tilt is smoothed using a LinearFilter configurated to
        // track a moving average. This is the number of taps in the moving average filter.
        // Each tap is 20ms long and therefore
        public static final int kMaxAbsoluteTiltMovingAverageTaps = 7 ;

        public class Resync {
            //
            // The position of the tilt must be less than this value, as measured by the
            // absolute encoder, in order to be considered in the resync position.
            //
            public static final double kPosThreshold = -30.0 ;

            //
            // The angular velocity of the tilt in degrees/second must be less than this
            // value to be considered for resync.
            public static final double kVelThreshold = 0.25 ;

            //
            // The absolute encoder tilt value and the motor tilt value, which were synced to be
            // equal when the robot was enabled, must be different by this value or more for the
            // resync to be applied.
            //
            public static final double kPosDiffThreshold = 10.0 ;
        }

        public class Positions {
            public static final double kStowed = -72.0 ;
            public static final double kStartTracking = -50.0 ;
            public static final double kCollect = 50.0 ;
            public static final double kTransfer = 0.0 ;
            public static final double kFinishTransfer = 0.0 ;            
            public static final double kTransferPosTol = 10.0 ;
            public static final double kTransferVelTol = 10.0 ;

            public static final double kShootNominal = -50.0 ;
            public static final double kEject = -50.0 ;
        }

        public class Real {
            public class MovementPIDSlot0 {
                public static final double kP = 8.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.30 ;
                public static final double kV = 0.1 ;
                public static final double kA = 0.0 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            };

            public class MotionMagic {
                // public static final double kV = 15 ;
                public static final double kV = kvfactor * 7.2727273 / 20.0  ;
                public static final double kA = kafactor * 7.2727273 / 20.0  ;
                public static final double kJ = kjfactor * 7.2727273 / 20.0  ;
            }
        } ;

        public class Simulated {
            public class MovementPIDSlot0 {
                public static final double kP = 1.0 ;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.25 ;
                public static final double kA = 0.0 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            };

            public class MotionMagic {
                public static final double kV = kvfactor * 7.2727273 / 20.0  ;
                public static final double kA = kafactor * 7.2727273 / 20.0  ;
                public static final double kJ = kjfactor * 7.2727273 / 20.0  ;
            }
        } ;


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
            1.37, -61.0,
            1.42, -59.0,
            1.72, -56.0,
            2.02, -53.0,
            2.319, -49.0,
            2.32, -67.0,
            2.62, -65.0,
            2.93, -62.0,
            3.23, -60.0,
            3.5, -58.5,
            3.9, -56.0,
            4.2, -54.5,
            4.5, -54.0
        } ;
    }

    public class Shooter {
        public static final double kVelocityTolerance = 2.5 ;
        public static final double kEjectForwardTime = 1.0 ;
        public static final double kEjectPauseTime = 0.5 ;
        public static final double kEjectReverseTime = 1.0 ;
        public static final double kEjectVelocity = 75.0 ;

        public static final double kAutoShootVelocityTol = 5.0 ;

        public static final double kTransferVelocity = 40.0 ;
        public static final double kTransferVelocityTol = 10.0 ;

        public static final double kTransferLength = 4.0 ;

        public static final double kShooterRevsPerMotorRev = 1.0 / 0.6 ;
        public static final double kShootMinVelocity = 0 ;
        public static final double kShootMaxVelocity = 90 ;

        public static final double kSimGearRatio = 0.6 ;
        public static final double kSimMotorLoad = 0.001 ;

        public static class Simulated {
            public static final double kP = 0.0;
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.125 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public static final double[] kPwlValues = new double[] {
            1.42, 75,
            1.72, 75,
            2.02, 75,
            2.319, 75,
            2.32, 80,
            2.62, 80,
            2.93, 80,
            3.23, 80,
            3.5, 80,
            4.5, 80
        } ;
    }

    public class Shooter1 {
        public static final int kMotorId = 3 ;
        public static final boolean kInvert = true ;
        public static final double kCurrentLimit = 80.0 ;

        public static class Real {
            public static class Pid {
                public static final double kP = 0.4;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.12727 ;
                public static final double kA = 0.01718 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.24131 ;
            }

            public static class MotionMagic {
                public static final double kV = 0.0;
                public static final double kA = 75.0 ;
                public static final double kJ = 1000.0 ;
            }
        }
    }

    public class Shooter2 {
        public static final int kMotorId = 4 ;
        public static final boolean kInvert = true ;
        public static final double kCurrentLimit = 80.0 ;

        public static class Real {
            public static class Pid {            
                public static final double kP = 0.4;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.12497 ;
                public static final double kA = 0.017806 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.21256 ;
            }

            public static class MotionMagic {
                public static final double kV = 0.0 ;
                public static final double kA = 75.0 ;
                public static final double kJ = 1000.0 ;
            }            
        }
    }

    public class NoteSensor {
        public static final int kChannel = 1 ;
        public static final boolean kInverted = true ;
    }
}
