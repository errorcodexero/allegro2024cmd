package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.xero1425.HolonomicPathFollower;
import org.xero1425.Pose2dWithRotation;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.tracker.Tracker;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
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

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }
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

        Pigeon2 p = getPigeon2() ;
        LimelightHelpers.SetRobotOrientation(limelight_name_, p.getYaw().getValueAsDouble(), p.getRate(), p.getPitch().getValueAsDouble(), 0.0, p.getRoll().getValueAsDouble(), 0.0) ;


        // addVisionMeasurement(tracker_.getVisionPose(), tracker_.getVisionTimeStamp()) ;

        if (follower_ != null) {
            follower_.execute() ;
            if (!follower_.isDriving()) {
                follower_ = null ;
            }
        }

        Logger.recordOutput("SwerveState", getState().ModuleStates) ;
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
