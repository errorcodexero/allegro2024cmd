package org.xero1425.simsupport;

public class SimEvent {
    public final double time_ ;
    public final String hardware_type_ ;
    public final String hardware_subtype_ ;
    public final int index_ ;
    public final Boolean bvalue_ ;
    public final Double dvalue_ ;
    public final Integer ivalue_ ;
    public final String svalue_ ;

    public SimEvent(double time, String type, String subtype, int index, boolean value) {
        time_ = time ;
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = value ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = null ;
    }

    public SimEvent(double time, String type, int index, boolean value) {
        time_ = time ;        
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = value ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = null ;        
    }    

    public SimEvent(double time, String type, String subtype, int index, double value) {
        time_ = time ;
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = value ;
        ivalue_ = null ;
        svalue_ = null ;
    }

    public SimEvent(double time, String type, int index, double value) {
        time_ = time ;        
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = value ;
        ivalue_ = null ;
        svalue_ = null ;        
    }

    public SimEvent(double time, String type, String subtype, int index, int value) {
        time_ = time ;        
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = value ;
        svalue_ = null ;        
    }

    public SimEvent(double time, String type, int index, int value) {
        time_ = time ;    
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = value ;
        svalue_ = null ;        
    }

    public SimEvent(double time, String type, String subtype, int index, String value) {
        time_ = time ;        
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = value ;        
    }

    public SimEvent(double time, String type, int index, String value) {
        time_ = time ;    
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = value ;        
    }    

    public SimEvent(String type, String subtype, int index, boolean value) {
        time_ = Double.NaN ;
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = value ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = null ;        
    }

    public SimEvent(String type, int index, boolean value) {
        time_ = Double.NaN ;        
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = value ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = null ;        
    }  

    public SimEvent(String type, String subtype, int index, double value) {
        time_ = Double.NaN ;            
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = value ;
        ivalue_ = null ;
        svalue_ = null ;        
    }

    public SimEvent(String type, int index, double value) {
        time_ = Double.NaN ;            
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = value ;
        ivalue_ = null ;
        svalue_ = null ;        
    }

    public SimEvent(String type, String subtype, int index, int value) {
        time_ = Double.NaN ;
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = value ;
        svalue_ = null ;        
    }

    public SimEvent(String type, int index, int value) {
        time_ = Double.NaN ;
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = value ;
        svalue_ = null ;        
    }   

    public SimEvent(String type, String subtype, int index, String value) {
        time_ = Double.NaN ;
        hardware_type_ = type ;
        hardware_subtype_ = subtype ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = value ;        
    }

    public SimEvent(String type, int index, String value) {
        time_ = Double.NaN ;
        hardware_type_ = type ;
        hardware_subtype_ = null ;
        index_ = index ;
        bvalue_ = null ;
        dvalue_ = null ;
        ivalue_ = null ;
        svalue_ = value ;        
    }     
}
