package frc.robot.automodes.competition;

import org.xero1425.XeroAutoCommand;
import org.xero1425.XeroRobot;

import frc.robot.AllegroContainer;

public class JustShootCommand extends XeroAutoCommand {
    public enum StartLocation {
        Center,
        AmpSide,
        SourceSide
    }

    public static class CenterShot {
        public static final double kLowManualTilt = 3.0 ;
        public static final double kLowManualTiltPosTol = 2.0 ;
        public static final double kLowManualTiltVelTol = 2.0 ;
        public static final double kLowManualUpDown = 35.0 ;
        public static final double kLowManualUpDownPosTol = 5.0 ;
        public static final double kLowManualUpDownVelTol = 5.0 ;
        public static final double kLowManualShooter = 65.0 ;
        public static final double kLowManualShooterVelTol = 5.0 ;
    }

    public static class AmpSideShot {
        public static final double kLowManualTilt = 3.0 ;
        public static final double kLowManualTiltPosTol = 2.0 ;
        public static final double kLowManualTiltVelTol = 2.0 ;
        public static final double kLowManualUpDown = 35.0 ;
        public static final double kLowManualUpDownPosTol = 5.0 ;
        public static final double kLowManualUpDownVelTol = 5.0 ;
        public static final double kLowManualShooter = 65.0 ;
        public static final double kLowManualShooterVelTol = 5.0 ;
    } 
    
    public static class SourceSideShot {
        public static final double kLowManualTilt = 3.0 ;
        public static final double kLowManualTiltPosTol = 2.0 ;
        public static final double kLowManualTiltVelTol = 2.0 ;
        public static final double kLowManualUpDown = 35.0 ;
        public static final double kLowManualUpDownPosTol = 5.0 ;
        public static final double kLowManualUpDownVelTol = 5.0 ;
        public static final double kLowManualShooter = 65.0 ;
        public static final double kLowManualShooterVelTol = 5.0 ;
    }    

    private StartLocation loc_ ;
    private AllegroContainer container_ ;

    private final static String desc = "This auto mode starts in any one of the three positions and just shoots the loaded note." ;    

    public JustShootCommand(XeroRobot robot, AllegroContainer container, StartLocation loc) {
        super(robot, "just-shoot-" + locToUserString(loc), desc) ;
        loc_ = loc ;
        container_ = container ;
    }

    private static String locToUserString(StartLocation loc) {
        switch(loc) {
            case Center:
                return "center" ;
            case AmpSide:
                return "amp-side" ;
            case SourceSide:
                return "source-side" ;
        }
        return "unknown" ;
    }

    @Override
    public void initialize() {
        switch(loc_) {
            case Center:
                doCenter() ;
                break ;
            case AmpSide:
                doAmpSide() ;
                break ;
            case SourceSide:
                doSourceSide() ;
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return container_.getIntakeShooter().isIdle() ;
    }

    private void doCenter() {
        container_.getIntakeShooter().manualShoot(
            CenterShot.kLowManualUpDown, CenterShot.kLowManualUpDownPosTol, CenterShot.kLowManualUpDownVelTol, 
            CenterShot.kLowManualTilt, CenterShot.kLowManualTiltPosTol, CenterShot.kLowManualTiltVelTol,
            CenterShot.kLowManualShooter, CenterShot.kLowManualShooterVelTol,
            false) ;
    }

    private void doAmpSide() {
        container_.getIntakeShooter().manualShoot(
            AmpSideShot.kLowManualUpDown, AmpSideShot.kLowManualUpDownPosTol, AmpSideShot.kLowManualUpDownVelTol, 
            AmpSideShot.kLowManualTilt, AmpSideShot.kLowManualTiltPosTol, AmpSideShot.kLowManualTiltVelTol,
            AmpSideShot.kLowManualShooter, AmpSideShot.kLowManualShooterVelTol,
            false) ;
    }    

    private void doSourceSide() {
        container_.getIntakeShooter().manualShoot(
            SourceSideShot.kLowManualUpDown, SourceSideShot.kLowManualUpDownPosTol, SourceSideShot.kLowManualUpDownVelTol, 
            SourceSideShot.kLowManualTilt, SourceSideShot.kLowManualTiltPosTol, SourceSideShot.kLowManualTiltVelTol,
            SourceSideShot.kLowManualShooter, SourceSideShot.kLowManualShooterVelTol,
            false) ;
    }       
}
