package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.HolonomicPathFollower;
import org.xero1425.Pose2dWithRotation;
import org.xero1425.XeroMath;
import org.xero1425.XeroRobot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.RobotConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final int kRecordModuleStates = (1  << 0) ;
    private static final int kDisplayAcquisition = (1 << 1) ;
    private static final int kRecordGyroYaw = (1 << 2) ;
    private static final int kDisplayRobotPose = (1 << 3) ;
    private static final int kDisplayEncoderValues = (1 << 4) ;
    private static final int kRecordModulePositions = (1 << 5) ;
    private static final int kRecordModuleTargets = (1 << 6) ;
    private int kDumpOutputSelected =   kRecordModuleStates | 
                                        kDisplayAcquisition | 
                                        kDisplayEncoderValues | 
                                        kDisplayRobotPose |
                                        kRecordModuleStates | 
                                        kRecordGyroYaw |
                                        kRecordModuleTargets ;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private String limelight_name_ ;

    // Path follower for this drive base
    private HolonomicPathFollower follower_ ;    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();

    private XeroRobot robot_ ;

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        CommandScheduler.getInstance().registerSubsystem(this);

        tareEverything();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        CommandScheduler.getInstance().registerSubsystem(this);

        tareEverything();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void setRobot(XeroRobot robot) {
        robot_ = robot ;
    }

    public void setLimelightName(String name) {
        limelight_name_ = name ;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    @Override
    public void periodic() {
        if (robot_ == null) {
            DriverStation.reportError("Robot object not set in swerve drivetrain", false) ;
        }

        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        Rotation2d robotHeading = getState().Pose.getRotation();
        LimelightHelpers.SetRobotOrientation(limelight_name_, robotHeading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0) ;
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name_) ;
        if (estimate.tagCount > 0) {
            addVisionMeasurement(estimate.pose, estimate.timestampSeconds) ;
        }

        if (follower_ != null) {
            follower_.execute() ;
            if (!follower_.isDriving()) {
                follower_ = null ;
            }
        }

        dumpOutput() ;
    }

    public void driveTo(String pathname, Pose2d[] imd, Pose2dWithRotation dest, double maxv, double maxa, double pre_rot_time, double pose_rot_time, double to) {
        follower_ = new HolonomicPathFollower(createHolonimicPathFollowerConfig());
        follower_.driveTo(pathname, imd, dest, maxv, maxa, pre_rot_time, pose_rot_time, to);
    }

    public double getPathDistance() {
        double ret = 0.0 ;

        if (follower_ != null) {
            ret = follower_.getDistance() ;
        }

        return ret ;
    }

    public boolean isFollowingPath() {
        return follower_ != null ;
    }

    private void dumpOutput() {
        if ((kDumpOutputSelected & kRecordModuleStates) == kRecordModuleStates) {
            var states = m_cachedState.ModuleStates ;
            Logger.recordOutput("ms:fl-a", XeroMath.normalizeAngleDegrees(states[0].angle.getDegrees())) ;
            Logger.recordOutput("ms:fr-a", XeroMath.normalizeAngleDegrees(states[1].angle.getDegrees())) ;
            Logger.recordOutput("ms:bl-a", XeroMath.normalizeAngleDegrees(states[2].angle.getDegrees())) ;
            Logger.recordOutput("ms:br-a", XeroMath.normalizeAngleDegrees(states[3].angle.getDegrees())) ;
            Logger.recordOutput("ms:fl-v", states[0].speedMetersPerSecond) ;
            Logger.recordOutput("ms:fr-v", states[1].speedMetersPerSecond) ;
            Logger.recordOutput("ms:bl-v", states[2].speedMetersPerSecond) ;
            Logger.recordOutput("ms:br-v", states[3].speedMetersPerSecond) ;
        }

        if ((kDumpOutputSelected & kRecordModuleTargets) == kRecordModuleTargets) {
            var states = m_cachedState.ModuleStates ;
            Logger.recordOutput("mt:fl-a", XeroMath.normalizeAngleDegrees(states[0].angle.getDegrees())) ;
            Logger.recordOutput("mt:fr-a", XeroMath.normalizeAngleDegrees(states[1].angle.getDegrees())) ;
            Logger.recordOutput("mt:bl-a", XeroMath.normalizeAngleDegrees(states[2].angle.getDegrees())) ;
            Logger.recordOutput("mt:br-a", XeroMath.normalizeAngleDegrees(states[3].angle.getDegrees())) ;
            Logger.recordOutput("mt:fl-v", states[0].speedMetersPerSecond) ;
            Logger.recordOutput("mt:fr-v", states[1].speedMetersPerSecond) ;
            Logger.recordOutput("mt:bl-v", states[2].speedMetersPerSecond) ;
            Logger.recordOutput("mt:br-v", states[3].speedMetersPerSecond) ;
        }        

        if ((kDumpOutputSelected & kDisplayAcquisition) == kDisplayAcquisition) {
            SmartDashboard.putNumber("good-daq", m_cachedState.SuccessfulDaqs) ;
            SmartDashboard.putNumber("bad-daq", m_cachedState.FailedDaqs) ;
        }

        if ((kDumpOutputSelected & kRecordGyroYaw) == kRecordGyroYaw) {
            Logger.recordOutput("yaw", XeroMath.normalizeAngleDegrees(getPigeon2().getYaw().getValueAsDouble())) ;
        }

        if ((kDumpOutputSelected & kDisplayRobotPose) == kDisplayRobotPose) {        
            SmartDashboard.putNumber("db-x", getState().Pose.getX()) ;
            SmartDashboard.putNumber("db-y", getState().Pose.getY()) ;
            SmartDashboard.putNumber("db-a", getState().Pose.getRotation().getDegrees()) ;  
        }

        if ((kDumpOutputSelected & kDisplayEncoderValues) == kDisplayEncoderValues) {
            SmartDashboard.putNumber("fl-enc", this.Modules[0].getDriveMotor().getPosition().getValueAsDouble()) ;
            SmartDashboard.putNumber("fr-enc", this.Modules[1].getDriveMotor().getPosition().getValueAsDouble()) ;
            SmartDashboard.putNumber("bl-enc", this.Modules[2].getDriveMotor().getPosition().getValueAsDouble()) ;
            SmartDashboard.putNumber("br-enc", this.Modules[3].getDriveMotor().getPosition().getValueAsDouble()) ;            
        }

        if ((kDumpOutputSelected & kRecordModulePositions) == kRecordModulePositions) {
            var states = m_modulePositions ;
            Logger.recordOutput("mp:fl-a", XeroMath.normalizeAngleDegrees(states[0].angle.getDegrees())) ;
            Logger.recordOutput("mp:fr-a", XeroMath.normalizeAngleDegrees(states[1].angle.getDegrees())) ;
            Logger.recordOutput("mp:bl-a", XeroMath.normalizeAngleDegrees(states[2].angle.getDegrees())) ;
            Logger.recordOutput("mp:br-a", XeroMath.normalizeAngleDegrees(states[3].angle.getDegrees())) ;
            Logger.recordOutput("mp:fl-v", states[0].distanceMeters) ;
            Logger.recordOutput("mp:fr-v", states[1].distanceMeters) ;
            Logger.recordOutput("mp:bl-v", states[2].distanceMeters) ;
            Logger.recordOutput("mp:br-v", states[3].distanceMeters) ;
        }        
    }

    private HolonomicPathFollower.Config createHolonimicPathFollowerConfig() {
        HolonomicPathFollower.Config cfg = new HolonomicPathFollower.Config() ;

        cfg.max_rot_velocity = SwerveConstants.kMaxRotationalSpeed ;
        cfg.max_rot_acceleration = SwerveConstants.kMaxRotationalAccel ;

        cfg.rot_p = RobotConstants.PathFollowing.RotCtrl.kP ;
        cfg.rot_i = RobotConstants.PathFollowing.RotCtrl.kI ;
        cfg.rot_d = RobotConstants.PathFollowing.RotCtrl.kD ;

        cfg.x_d = RobotConstants.PathFollowing.XCtrl.kD ;
        cfg.x_i = RobotConstants.PathFollowing.XCtrl.kI ;
        cfg.x_p = RobotConstants.PathFollowing.XCtrl.kP ;

        cfg.y_d = RobotConstants.PathFollowing.YCtrl.kD ;
        cfg.y_i = RobotConstants.PathFollowing.YCtrl.kI ;
        cfg.y_p = RobotConstants.PathFollowing.YCtrl.kP ;

        cfg.xytolerance = RobotConstants.PathFollowing.kXYTolerance ;
        cfg.rot_tolerance = Rotation2d.fromDegrees(RobotConstants.PathFollowing.kAngleTolerance) ;

        cfg.pose_supplier = () -> getState().Pose ;
        cfg.output_consumer = (ChassisSpeeds spd) -> setControl(new ApplyChassisSpeeds().withSpeeds(spd)) ;

        return cfg ;

    }    

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }    
}
