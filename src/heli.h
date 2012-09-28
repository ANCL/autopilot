/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * 
 * This file is part of ANCL Autopilot.
 * 
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 * 
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 * 
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#ifndef HELI_H_
#define HELI_H_

/** This global namespace contains the pulses and priority settings in this project. This helps:
      - consolidate all pulses used in the project, so is easier to define, manage and expand if needed.
      - being of global scope makes it easier to be used in any sub-scope part of the project.
    Range of pulses that can be used 0 - 127 (128 pulse codes).
    Documentation recommends using non-negative pulse codes.

    Thread priorities can be set to a value between 0 and 255, 0 being lowest and 255 highest.
 */

/* STL Headers */
#include <iostream>

/* C Headers */
#include <cmath>

/* Boost Headers */
#include <boost/signals2.hpp>

/* Project Headers */


#include <sys/neutrino.h>


namespace heli
{
const short mainThreadPulseCode = _PULSE_CODE_MINAVAIL;
const short outputBoardPulseCode = _PULSE_CODE_MINAVAIL + 1;
//  const short LogFilePulseCode = _PULSE_CODE_MINAVAIL + 2;
const short inputBoardPulseCode = _PULSE_CODE_MINAVAIL + 3;
const short imuUpdatePulseCode = _PULSE_CODE_MINAVAIL + 4;
const short ahrsUpdatePulseCode = _PULSE_CODE_MINAVAIL + 5;
const short QGCLinkSendPulseCode = _PULSE_CODE_MINAVAIL + 6;
const short servo_switch_pulse_code = _PULSE_CODE_MINAVAIL + 7;


// priorities list (lowest = 0, highest = 255)
const unsigned int mainThreadPriority = 100;
const unsigned int outUpdatePriority = 99;
const unsigned int imuUpdatePriority = 98;
const unsigned int ahrsUpdatePriority = 97;
const unsigned int qgcSendPriority = 85;
const unsigned int servo_switch_send_priority = 99;

// controller logs
const std::string LOG_ATTITUDE_ERROR = "Attitude Error";
const std::string LOG_ATTITUDE_REFERENCE = "Pilot Attitude Reference";
const std::string LOG_POSITION_REFERENCE = "Position Reference (Nav Frame)";
const std::string LOG_POSITION_ERROR = "Position Error (Body Frame)";
const std::string LOG_NORMALIZED_OUTPUTS = "Normalized Outputs";
const std::string LOG_TRANS_ATTITUDE_REF = "Translation PID Attitude Reference";

//const std::string LOG_MICROSTRAIN_AHRS = "Microstrain AHRS";
//const std::string LOG_MICROSTRAIN_AHRS_UNFILTERED = "Microstrain AHRS Unfiltered";

// novatel logs
const std::string LOG_NOVATEL_GPS = "Novatel GPS (Invalid Solutions Removed)";
const std::string LOG_NOVATEL_GPS_ALL = "Novatel GPS (All Measurements)";

// servo logs
const std::string LOG_INPUT_PULSE_WIDTHS = "Input Pulse Widths";
const std::string LOG_SCALED_INPUTS = "Scaled Inputs";
const std::string LOG_OUTPUT_PULSE_WIDTHS = "Output Pulse Widths";
const std::string LOG_INPUT_RPM = "Engine RPM";

// gx3 logs
const std::string LOG_LLH_POS = "GX3 Estimated LLH Position";
const std::string LOG_NED_VEL = "GX3 Estimated NED Velocity";
const std::string LOG_ORIENTATION = "GX3 Nav Orientation Matrix";
const std::string LOG_ANG_RATE = "GX3 Nav Angular Rates";
const std::string LOG_ANG_RATE_FILTERED = "GX3 Nav Angular Rates Filtered";
const std::string LOG_EULER = "GX3 Nav Euler Angles";
const std::string Log_AHRS_Euler = "GX3 AHRS Euler Angles";
const std::string Log_AHRS_Ang_Rate = "GX3 AHRS Angular Rates";
const std::string Log_AHRS_Ang_Rate_Filtered = "GX3 AHRS Angular Rates Filtered";

void shutdown(int signumber);

// To switch between AHRS modes
enum AHRS_MODE
{
	AHRS_INIT = 0,
	AHRS_COMPUT,
};

enum PILOT_MODE
{
	PILOT_MANUAL,
	PILOT_AUTO,
	PILOT_UNKNOWN,
	NUM_PILOT_MODES
};

enum NAVIGATION_SOURCE
{
	AHRS,
	GPSINS,
	MICROSTRAIN
};

// Used by QGCLink receive to detect a parameter change.
enum COMPONENT_ID
{
	NAVFILTER_ID = 10,
	CONTROLLER_ID = 20,
	GX3_ID = 30,
	SERVO_SWITCH_ID = 40,
	RADIO_CAL_ID = 50,
	NOVATEL_ID = 60,
	NUM_COMPONENT_IDS
};

/// Enumeration constant to represent autopilot mode
enum AUTOPILOT_MODE
{
	MODE_DIRECT_MANUAL,
	MODE_SCALED_MANUAL,
	MODE_AUTOMATIC_CONTROL,
	NUM_AUTOPILOT_MODES
};

enum Controller_Mode
{
	Mode_Attitude_Stabilization_PID,
	Mode_Position_Hold_PID,
	Mode_Position_Hold_SBF,
	Num_Controller_Modes
};

enum GPS_MODE
{
	MODE_GPS_UNINITIALIZED,
	MODE_GPS_INITIALIZED
};

/**
 *  \enum Channel
 */
enum Channel
{
	/** Aileron (Roll) */
    CH1=0,
	/** Elevator (Pitch) */
    CH2,
    /** Throttle */
    CH3,
    /** Rudder (Yaw) */
    CH4,
    /** Gyro */
    CH5,
    /** Collective Pitch */
    CH6,
    /** AUX */
    CH7,
    /** Ready Signal */
    CH8,
    /** Not connected*/
    CH9
};

const std::string calibration_filename = "/etc/autopilot/Calibration.xml";
const std::string controller_param_filename = "/etc/autopilot/Controller_Parameters.xml";

}
#endif

// todo include priority codes here.
