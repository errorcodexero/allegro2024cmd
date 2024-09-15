package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveRotateToAngle extends Command {

    private static final double kP = 0.3 ;
    private static final double kI = 0.0 ;
    private static final double kD = 0.0 ;

    private static final double kDefaultPostol = 5.0 ;
    private static final double kDefaultVeltol = 5.0 ;

    private PIDController pid_ ;

    private CommandSwerveDrivetrain db_;
    private DoubleSupplier angle_supplier_ ;
    private boolean is_finished_ ;
    private double postol_ ;
    private double veltol_ ;

    public SwerveRotateToAngle(CommandSwerveDrivetrain db, DoubleSupplier angSupplier, double postol, double veltol) {
        addRequirements(db);

        setName("swerve-rotate-to-angle");

        db_ = db;
        angle_supplier_ = angSupplier;

        postol_ = postol ;
        veltol_ = veltol ;
    }

    public SwerveRotateToAngle(CommandSwerveDrivetrain db, DoubleSupplier angSupplier) {
        this(db, angSupplier, kDefaultPostol, kDefaultVeltol) ;
    }

    public SwerveRotateToAngle withPositionTolerance(double v) {
        postol_ = v ;
        return this ;
    }

    public SwerveRotateToAngle withVelocityTolerance(double v) {
        veltol_ = v ;
        return this ;
    }

    @Override
    public void initialize() {
        pid_ = new PIDController(kP, kI, kD) ;
        is_finished_ = false ;

    }

    @Override
    public void execute() {
        double target = angle_supplier_.getAsDouble() ;
        double current = db_.getState().Pose.getRotation().getDegrees() ;
        double rotvel = pid_.calculate(current, target) ;

        double current_angular_velocity = db_.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() ;
        is_finished_ = Math.abs(current - target) < postol_ && Math.abs(current_angular_velocity) < veltol_ ;

        if (!is_finished_) {
            db_.setControl(new ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, rotvel))) ;
        }
        else {
            db_.setControl(new SwerveDriveBrake()) ;
        }

        Logger.recordOutput("rotdb:target", target) ;
        Logger.recordOutput("rotdb:position", db_.getState().Pose.getRotation().getDegrees())  ;
        Logger.recordOutput("rotdb:velocity", current_angular_velocity) ;
        Logger.recordOutput("rotdb:is-finished", is_finished_) ;
    }

    @Override
    public boolean isFinished() {
        return is_finished_ ;
    }
}
