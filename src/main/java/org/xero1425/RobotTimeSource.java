package org.xero1425 ;

import edu.wpi.first.wpilibj.Timer ;

/// \file

/// \brief This is a time source class tha implements the time source interface requried by the meessage logger. 
/// This class returns the time from the WPILib FPGA time stamp as a time source for the message logger.
public class RobotTimeSource implements MessageTimeSource
{
    /// \brief Create the new time source
    public RobotTimeSource() {
    }

    /// \brief Returns the time based on the FPGA time from the WPILib APIs.
    /// returns the time based on the FPGA time from the WPILib APIs.
    public double getTime() {
        return Timer.getFPGATimestamp() ;
    }
}
