package frc.robot.subsystems.gamepiecetracker;

import java.util.Arrays;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.base.LimelightHelpers.LimelightTarget_Detector;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;

public class GamePieceTracker extends XeroSubsystem {
    private String llname_ ;
    private LimelightTarget_Detector[] game_pieces_ ;
    private double confidence_ ;
    private double camangle_ ;
    private double distance_ ;
    private double height_ ;

    public GamePieceTracker(XeroRobot robot, String llname, double confidence, double height, double camangle) {
        super(robot, "gamepiecetracker");

        this.llname_ = llname ;
        this.confidence_ = confidence ;
        this.camangle_ = camangle ;
        this.height_ = height ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        return null ;
    }

    @Override
    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    public boolean seesGamePieces() {
        if (game_pieces_ == null || game_pieces_.length == 0)
            return false ;

        return true ;
    }

    public LimelightTarget_Detector[] getGamePieces() {
        return game_pieces_ ;
    }

    public double getDistanceClosestGamePiece() {
        if (!seesGamePieces())
            return Double.POSITIVE_INFINITY;

        return distance_ ;
    }

    public double getAngleClosestGamePiece() {
        if (!seesGamePieces())
            return Double.NaN ;

        return game_pieces_[0].tx ;
    }

    public LimelightTarget_Detector getClosestGamePiece() {
        LimelightTarget_Detector ret = null ;

        if (game_pieces_ != null && game_pieces_.length > 0 && game_pieces_[0].confidence > this.confidence_) {
            ret = game_pieces_[0] ;
        }

        return ret ;
    }
    
    @Override
    public void periodic() {
        startPeriodic();

        game_pieces_ = LimelightHelpers.getLatestResults(llname_).targets_Detector ;


        Arrays.sort(game_pieces_, GamePieceTracker::compareGamePieces) ;

        distance_ = -Math.tan(this.camangle_ + game_pieces_[0].ty) * this.height_ ;

        Logger.recordOutput("gptracker:number", game_pieces_.length) ;
        Logger.recordOutput("gptracker:distance", distance_);

        endPeriodic();
    }

    private static int compareGamePieces(LimelightTarget_Detector a, LimelightTarget_Detector b) {
        int ret = 0 ;

        if (a.tx < b.tx)
            ret = -1 ;
        else if (a.tx > b.tx)
            ret = 1 ;
        else
            ret = 0 ;

        return ret ;
    }
}
