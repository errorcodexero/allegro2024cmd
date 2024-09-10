package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.revrobotics.CANSparkBase;

public class VisionSubsystem extends XeroSubsystem {
    private SwerveDrivetrain db_ ;
    private boolean enabled_ ;
    private VisionIO io_ ;
    private VisionInputsAutoLogged inputs_;

    public VisionSubsystem(XeroRobot robot, SwerveDrivetrain db, String name) {
        super(robot, "vision");
        enabled_ = false ;

        io_ = new VisionIOLimelight(name) ;
        inputs_ = new VisionInputsAutoLogged() ;
        db_ = db ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }    

    public void enable(boolean b) {
        enabled_ = b ;
    }

    @Override
    public void periodic() {
        periodicStart();

        io_.updateInputs(inputs_);

        // Logger.processInputs("vision", inputs_);

        if (enabled_) {

            Logger.recordOutput("vision-enabled", true) ; 

            if (inputs_.tagCount > 0) {
                db_.addVisionMeasurement(inputs_.pose, inputs_.timestampSeconds) ;
            }         
        } else {
            Logger.recordOutput("vision-enabled", false) ;
        }

        periodicEnd();
    }

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    public List<CANSparkBase> getRevRoboticsMotors() {
        return null ;
    }    
}
