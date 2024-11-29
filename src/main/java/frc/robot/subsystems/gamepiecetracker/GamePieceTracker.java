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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;

public class GamePieceTracker extends XeroSubsystem {
    private String llname_ ;
    private LimelightTarget_Detector[] game_pieces_ ;
    private double confidence_ ;
    private double camangle_ ;
    private double distance_ ;
    private double height_ ;
    private MedianFilter median_filter_y_ ;
    private MedianFilter median_filter_x_ ;
    private LinearFilter average_filter_ ;
    private double tx_ ;

    public GamePieceTracker(XeroRobot robot, String llname, double confidence, double height, double camangle) {
        super(robot, "gamepiecetracker");

        this.llname_ = llname ;
        this.confidence_ = confidence ;
        this.camangle_ = camangle ;
        this.height_ = height ;

        median_filter_y_ = new MedianFilter(3) ;
        median_filter_x_ = new MedianFilter(3) ;
        average_filter_ = LinearFilter.movingAverage(3) ;
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

        return tx_ ;
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
        double ty ;

        startPeriodic();

        game_pieces_ = LimelightHelpers.getLatestResults(llname_).targets_Detector ;


        Arrays.sort(game_pieces_, GamePieceTracker::compareGamePieces) ;

        if (seesGamePieces()) {
            ty = median_filter_y_.calculate(game_pieces_[0].ty) ;
            tx_ = median_filter_x_.calculate(game_pieces_[0].tx) ;

            ty = game_pieces_[0].ty ;
            tx_ = game_pieces_[0].tx ;

            ty = game_pieces_[0].ty ;
            distance_ =  this.height_ / Math.tan(Math.toRadians(this.camangle_ + ty)) ;

            Logger.recordOutput("gptracker:tx", game_pieces_[0].tx) ;
            Logger.recordOutput("gptracker:ty", game_pieces_[0].ty) ;
        }
        else {
            distance_ = Double.POSITIVE_INFINITY ;
            ty = Double.NaN;
        }

        Logger.recordOutput("gptracker:number", game_pieces_.length) ;
        Logger.recordOutput("gptracker:distance", distance_);
        Logger.recordOutput("gptracker:effangle", this.camangle_ + ty) ;

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
