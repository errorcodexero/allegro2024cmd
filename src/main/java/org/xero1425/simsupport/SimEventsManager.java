package org.xero1425.simsupport;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.littletonrobotics.junction.Logger;
import org.xero1425.MessageLogger;
import org.xero1425.MessageType;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class SimEventsManager {
    private static final String button_prefix = "button-" ;
    private static final String axis_prefix = "axis-" ;

    private MessageLogger logger_;
    private boolean simerror_ ;
    private List<SimEvent> initialization_;
    private List<SimEvent> events_;
    private double start_time_ ;
    private String filename_ ;

    public SimEventsManager(MessageLogger logger) {
        logger_ = logger;
        events_ = new ArrayList<>();
        initialization_ = new ArrayList<>();

        start_time_ = Double.NaN ;
    }

    public boolean readEventsFile(String filename) {
        filename_ = filename ;
        JSONObject jobj = JsonReader.readFile(filename, logger_);
        if (jobj == null) {
            logger_.startMessage(MessageType.Error);
            logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
            logger_.add("error occurred while reading file");
            simerror_ = true ;
            return false;
        }

        if (jobj.containsKey("initialization")) {
            Object obj = jobj.get("initialization");
            if (!(obj instanceof JSONArray)) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("initialization is not an array");
                simerror_ = true ;                
                return false;
            }
            JSONArray arr = (JSONArray) obj;
            if (!parseEvents(filename, arr, initialization_, false)) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("error occurred while reading initialization events");
                simerror_ = true ;                
                return false;
            }
        }

        if (jobj.containsKey("simulation")) {
            Object obj = jobj.get("simulation");
            if (!(obj instanceof JSONArray)) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("simulation is not an array");
                simerror_ = true ;                
                return false;
            }
            JSONArray arr = (JSONArray) obj;
            if (!parseEvents(filename, arr, events_, true)) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("error occurred while reading simulation events");
                simerror_ = true ;                
                return false;
            }
        }

        //
        // Now finally sort the list based on the time
        //
        Collections.sort(events_, new Comparator<SimEvent>() {
            @Override
            public int compare(SimEvent o1, SimEvent o2) {
                if (o1.time_ < o2.time_)
                    return -1;
                if (o1.time_ > o2.time_)
                    return 1;
                return 0;
            }
        });

        return true;
    }

    private boolean parseEvents(String filename, JSONArray arr, List<SimEvent> events, boolean needtime) {
        for (Object obj : arr) {
            if (!(obj instanceof JSONObject)) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("event is not an object");
                simerror_ = true ;                
                return false;
            }

            JSONObject jobj = (JSONObject) obj;
            if (!jobj.containsKey("type")) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("event does not have a type");
                simerror_ = true ;                  
                return false;
            }
            String type = (String) jobj.get("type");
            if (type.equals("input")) {
                if (!parseEvent(filename, jobj, events, needtime)) {
                    simerror_ = true ;                    
                    return false;
                }
            }
        }

        return true;
    }

    private boolean parseEvent(String filename, JSONObject jobj, List<SimEvent> events, boolean needtime) {
        double time = Double.NaN ;
        if (needtime) {
            if (!jobj.containsKey("time")) {                
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("event does not have a time") ;
                return false;
            }            
            time = (double)jobj.get("time") ;
        }
        else {
            if (jobj.containsKey("time")) {                
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("event has a time value for an intialization entry") ;
                return false;
            }                  
        }

        if (!jobj.containsKey("hardware")) {
                logger_.startMessage(MessageType.Error);
                logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
                logger_.add("event does not have a hardware field ") ;
                return false;                
        }

        JSONObject hardware = (JSONObject)jobj.get("hardware") ;

        if (!hardware.containsKey("type")) {
            logger_.startMessage(MessageType.Error);
            logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
            logger_.add("event does not have a hardware type") ;
            return false;
        }
        String hwtype = (String)hardware.get("type") ;
        String hwsubtype = null ;

        if (hardware.containsKey("subtype")) {
            hwsubtype = (String)hardware.get("subtype") ;
        }

        int hwindex = (int)(long)hardware.get("index") ;

        if (!hardware.containsKey("value")) {
            logger_.startMessage(MessageType.Error);
            logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
            logger_.add("event does not have a value") ;
            return false;
        }

        Object value = hardware.get("value") ;
        SimEvent ev = null ;
        
        if (value instanceof Boolean) {
            if (needtime) {
                ev = new SimEvent(time, hwtype, hwsubtype, hwindex, (Boolean)value) ;
            }
            else {
                ev = new SimEvent(hwtype, hwsubtype, hwindex, (Boolean)value) ;
            }
        } else if (value instanceof Double) {
            if (needtime) {
                ev = new SimEvent(time, hwtype, hwsubtype, hwindex, (Double)value) ;
            }
            else {
                ev = new SimEvent(hwtype, hwsubtype, hwindex, (Double)value) ;
            }                
        } else if (value instanceof Long) {
            if (needtime) {
                long v = (long)value ;                
                ev = new SimEvent(time, hwtype, hwsubtype, hwindex, (int)v) ;
            }
            else {
                long v = (long)value ;
                ev = new SimEvent(hwtype, hwsubtype, hwindex, (int)v);
            }    
        } else if (value instanceof String) {
            if (needtime) {
                ev = new SimEvent(time, hwtype, hwsubtype, hwindex, (String)value) ;
            }
            else {
                ev = new SimEvent(hwtype, hwsubtype, hwindex, (String)value) ;
            }                           
        } else {
            logger_.startMessage(MessageType.Error);
            logger_.add("cannot read events file ").addQuoted(filename).add(" - ");
            logger_.add("event value is not a boolean, double, or integer") ;
            return false;
        }

        if (ev != null) {
            events.add(ev) ;
        }

        return true ;
    }

    private Integer mapSubtypeToButtonNumber(String subtype) {
        Integer ret = null ;

        if (subtype.startsWith(button_prefix)) {
            ret = Integer.decode(subtype.substring(button_prefix.length())) ;
        }
        else if (subtype.equals("right-bumper")) {
            ret = Integer.valueOf(XboxController.Button.kRightBumper.value) ;
        }
        else if (subtype.equals("left-bumper")) {
            ret = Integer.valueOf(XboxController.Button.kLeftBumper.value) ; 
        }
        else if (subtype.equals("left-stick")) {
            ret = Integer.valueOf(XboxController.Button.kLeftStick.value) ; 
        }
        else if (subtype.equals("right-stick")) {
            ret = Integer.valueOf(XboxController.Button.kRightStick.value) ; 
        }
        else if (subtype.equals("a")) {
            ret = Integer.valueOf(XboxController.Button.kA.value) ; 
        }
        else if (subtype.equals("b")) {
            ret = Integer.valueOf(XboxController.Button.kB.value) ; 
        }
        else if (subtype.equals("x")) {
            ret = Integer.valueOf(XboxController.Button.kX.value) ; 
        }
        else if (subtype.equals("y")) {
            ret = Integer.valueOf(XboxController.Button.kY.value) ; 
        }
        else if (subtype.equals("back")) {
            ret = Integer.valueOf(XboxController.Button.kBack.value) ; 
        }
        else if (subtype.equals("start")) {
            ret = Integer.valueOf(XboxController.Button.kStart.value) ;
        }
        return ret;
    }

    private Integer mapSubtypeToAxisNumber(String subtype) {
        Integer ret = null ;

        if (subtype.startsWith(axis_prefix)) {
            ret = Integer.decode(subtype.substring(axis_prefix.length())) ;
        }
        else if (subtype.equals("left-x")) {
            ret = Integer.valueOf(XboxController.Axis.kLeftX.value) ;
        }
        else if (subtype.equals("left-y")) {
            ret = Integer.valueOf(XboxController.Axis.kLeftY.value) ;
        }
        else if (subtype.equals("right-x")) {
            ret = Integer.valueOf(XboxController.Axis.kRightX.value) ;
        }
        else if (subtype.equals("right-y")) {
            ret = Integer.valueOf(XboxController.Axis.kRightY.value) ;
        }
        else if (subtype.equals("left-trigger")) {
            ret = Integer.valueOf(XboxController.Axis.kLeftTrigger.value) ;
        }
        else if (subtype.equals("right-trigger")) {
            ret = Integer.valueOf(XboxController.Axis.kRightTrigger.value) ;
        }
        
        return ret ;
    }    

    private void processOneEvent(double now, SimEvent ev) {
        if (ev.hardware_type_.equals("dinput")) {
            DIODataJNI.setIsInput(ev.index_, true) ;
            DIODataJNI.setValue(ev.index_, ev.bvalue_) ;
        }
        else if (ev.hardware_type_.equals("ainput")) {
            AnalogInDataJNI.setVoltage(ev.index_, ev.dvalue_);
        }
        else if (ev.hardware_type_.equals("driverstation")) {
            if (ev.hardware_subtype_.equals("mode")) {
                if (ev.svalue_.equals("teleop")) {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);
                    DriverStationSim.setEnabled(true);
                    DriverStationSim.notifyNewData();
                }
                else if (ev.svalue_.equals("auto")) {
                    DriverStationSim.setAutonomous(true);
                    DriverStationSim.setTest(false);
                    DriverStationSim.setEnabled(true);
                    DriverStationSim.notifyNewData();                    
                }                
            }
            else {
                logger_.startMessage(MessageType.Error) ;
                logger_.add("event has an driverstation type but unknown subtype") ;
                logger_.add("subtype", ev.hardware_subtype_) ;
                logger_.endMessage() ;
                simerror_ = true ;                
                return ;                
            }
        }
        else if (ev.hardware_type_.equals("oidevice")) {
            if (ev.hardware_subtype_.equals("button-count")) {
                DriverStationDataJNI.setJoystickButtonCount(ev.index_, ev.ivalue_) ;
            }
            else if (ev.hardware_subtype_.equals("axis-count")) {
                DriverStationDataJNI.setJoystickAxisCount(ev.index_, ev.ivalue_) ;
            }
            else {
                Integer value = mapSubtypeToButtonNumber(ev.hardware_subtype_) ;
                if (value != null) {
                    DriverStationDataJNI.setJoystickButton(ev.index_, value, ev.bvalue_) ;
                    DriverStationSim.notifyNewData();
                }
                else {
                    value = mapSubtypeToAxisNumber(ev.hardware_subtype_) ;
                    if (value != null) {
                        DriverStationDataJNI.setJoystickAxis(ev.index_, value, ev.dvalue_);
                        DriverStationSim.notifyNewData();
                    }
                    else {
                        logger_.startMessage(MessageType.Error) ;
                        logger_.add("event has an oidevice type but unknown subtype") ;
                        logger_.add("subtype", ev.hardware_subtype_) ;
                        logger_.endMessage() ;
                        simerror_ = true ;                
                        return ;
                    }
                }
            }
        }
        else if (ev.hardware_type_.equals("control")) {
            if (ev.hardware_subtype_.equals("exit")) {
                System.exit(0) ;
            }
            else {
                logger_.startMessage(MessageType.Error) ;
                logger_.add("event has an control type but unknown subtype") ;
                logger_.add("subtype", ev.hardware_subtype_) ;
                logger_.endMessage() ;
                simerror_ = true ;                
                return ;                
            }
        }
        else {
                logger_.startMessage(MessageType.Error) ;
                logger_.add("an unknown event type was found    ") ;
                logger_.add("hwtype", ev.hardware_type_) ;
                logger_.endMessage() ;      
                simerror_ = true ;                      
        }
    }

    public void processEvents(double now) {
        if (Double.isNaN(start_time_)) {
            start_time_ = now ;
        }

        while(events_.size() > 0) {
            SimEvent ev = events_.get(0) ;
            if (ev.time_ + start_time_ > now)
                break ;

            processOneEvent(now, ev);
            events_.remove(0) ;
        }

        if (simerror_)
            Logger.recordOutput("simfile", "**ERROR**") ;
        else
            Logger.recordOutput("simfile", filename_) ;
    }

    public void initialize() {
        for(SimEvent ev : initialization_) {
            processOneEvent(0.0, ev) ;
        }
    }
}
