package org.xero1425.simulator.models;

import org.xero1425.base.ISubsystemSim;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveModel extends SimulationModel {
    private static final String SubsystemNameProperty = "subsystem-name" ;

    private String subname_ ;
    private CommandSwerveDrivetrain db_ ;

    public SwerveDriveModel(final SimulationEngine engine, final String model, final String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) {
        try {
            subname_ = getStringProperty(SubsystemNameProperty) ;
        }
        catch(Exception ex) {
            final MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("model ").addQuoted(getModelName()) ;
            logger.add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" does not have a '").add(SubsystemNameProperty).add("' property") ;
            logger.endMessage() ;
            return false ;
        }
        setCreated();
        return true ;
    }

    @Override
    public void run(final double dt) {
        if (subname_ != null && db_ == null) {
            SimulationEngine engine = getEngine() ;
            ISubsystemSim sim = engine.getRobot().getSubsystemByName(subname_) ;
            if (sim == null) {
                engine.getMessageLogger().startMessage(MessageType.Error) ;
                engine.getMessageLogger().add("could not find subsystem '") ;
                engine.getMessageLogger().add(subname_) ;
                engine.getMessageLogger().add("' for swerve drive model '") ;
                engine.getMessageLogger().add(getInstanceName()) ;
                engine.getMessageLogger().add("'") ;
                engine.getMessageLogger().endMessage() ;
                return ;
            }
            db_ = (CommandSwerveDrivetrain)sim ;
            if (db_ == null) {
                engine.getMessageLogger().startMessage(MessageType.Error) ;
                engine.getMessageLogger().add("subsystem '") ;
                engine.getMessageLogger().add(subname_) ;
                engine.getMessageLogger().add("' is not of type CommandSwerveDrivetrain for the swerve drive model '") ;
                engine.getMessageLogger().add(getInstanceName()) ;
                engine.getMessageLogger().add("'") ;
                engine.getMessageLogger().endMessage() ;
            }        
        }
    }

    @Override
    public boolean processEvent(final String name, final SettingsValue value) {
        if (name.equals("pose") && db_ != null) {
            if (!value.isString())
            {
                final MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a string (should contains three doubles separated by commas)").endMessage();                
                return false ;
            }

            try {
                String[] parts = value.getString().split(",") ;
                if (parts.length != 3) {
                    final MessageLogger logger = getEngine().getMessageLogger() ;
                    logger.startMessage(MessageType.Error) ;
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(" event name ").addQuoted(name);
                    logger.add(" value is not a string containing three doubles separated by commas").endMessage() ;
                    return false ;
                }

                double x = Double.parseDouble(parts[0]) ;
                double y = Double.parseDouble(parts[1]) ;
                double angle = Double.parseDouble(parts[2]) ;
                db_.seedFieldRelative(new Pose2d(x, y, Rotation2d.fromDegrees(angle))) ;
            } catch (Exception ex) {
                final MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a string containing three doubles separated by commas").endMessage() ;
                return false ;
            }
        }
        return true ;
    }
}
