package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveRotateToAngle extends Command {
    private CommandSwerveDrivetrain db_;
    private DoubleSupplier angle_supplier_ ;
    private SwerveRequest.ApplyChassisSpeeds chassis_speeds_ ;
    private boolean is_finished_ ;
    private double postol_ ;
    private double veltol_ ;

    public SwerveRotateToAngle(CommandSwerveDrivetrain db, DoubleSupplier angSupplier) {
        addRequirements(db);

        db_ = db;
        angle_supplier_ = angSupplier;
        chassis_speeds_ = new SwerveRequest.ApplyChassisSpeeds();

        postol_ = 5.0 ;
        veltol_ = 5.0 ;
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
        is_finished_ = false ;
    }

    @Override
    public void execute() {
        double target_angle = angle_supplier_.getAsDouble() ;
        double target_angular_velocity = target_angle * SwerveConstants.kRotateP ;

        chassis_speeds_.withSpeeds(new ChassisSpeeds(0, 0, target_angular_velocity)) ;

        double current_angular_velocity = db_.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() ;
        is_finished_ = Math.abs(target_angle) < postol_ && Math.abs(current_angular_velocity) < veltol_ ;

        Logger.recordOutput("db-targ-angle", target_angle) ;
        Logger.recordOutput("db-targ-ang-vel", target_angular_velocity) ;
        Logger.recordOutput("db-rot-is-finished", is_finished_) ;
    }

    @Override
    public boolean isFinished() {
        return is_finished_ ;
    }
}
