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

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class SimEventsManager {
    private MessageLogger logger_;
    private boolean simerror_ ;
    private List<SimEvent> initialization_;
    private List<SimEvent> events_;

    public SimEventsManager(MessageLogger logger) {
        logger_ = logger;
        events_ = new ArrayList<>();
        initialization_ = new ArrayList<>();
        simerror_ = false ;

        DriverStationDataJNI.setJoystickButtonCount(0, 16) ;
        DriverStationDataJNI.setJoystickAxisCount(0, 8) ;
        DriverStationDataJNI.setJoystickButtonCount(2, 16) ;
    }

    public boolean readEventsFile(String filename) {
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

    public boolean parseEvents(String filename, JSONArray arr, List<SimEvent> events, boolean needtime) {
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
        } else if (value instanceof Integer) {
            if (needtime) {
                ev = new SimEvent(time, hwtype, hwsubtype, hwindex, (Integer)value) ;
            }
            else {
                ev = new SimEvent(hwtype, hwsubtype, hwindex, (Integer)value) ;
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

    private void processOneEvent(double now, SimEvent ev) {
        if (ev.hardware_type_.equals("dinput")) {
            DIODataJNI.setIsInput(ev.index_, true) ;
            DIODataJNI.setValue(ev.index_, ev.bvalue_) ;
        }
        else if (ev.hardware_type_.equals("driverstation")) {
            if (ev.hardware_subtype_.equals("mode")) {
                if (ev.svalue_.equals("teleop")) {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);
                    DriverStationSim.setEnabled(true);
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
        else if (ev.hardware_type_.equals("xboxgamepad")) {
            if (ev.hardware_subtype_.equals("right-bumper")) {
                DriverStationDataJNI.setJoystickButton(ev.index_, 6, ev.bvalue_) ;
                DriverStationSim.notifyNewData();                
            }
            else {
                logger_.startMessage(MessageType.Error) ;
                logger_.add("event has an xboxgamepad type but unknown subtype") ;
                logger_.add("subtype", ev.hardware_subtype_) ;
                logger_.endMessage() ;
                simerror_ = true ;                
                return ;
            }
        }
        else if (ev.hardware_type_.equals("oidevice")) {
            int button = Integer.parseInt(ev.hardware_subtype_) ;
            DriverStationDataJNI.setJoystickButton(ev.index_, button, ev.bvalue_) ;
            DriverStationSim.notifyNewData();              
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
        while(events_.size() > 0) {
            SimEvent ev = events_.get(0) ;
            if (ev.time_ > now)
                break ;

            processOneEvent(now, ev);
            events_.remove(0) ;
        }

        Logger.recordOutput("simerror", simerror_) ;
    }
}
