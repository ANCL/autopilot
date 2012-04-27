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

#ifndef GPS_TIME_H_
#define GPS_TIME_H_

/* C Headers */
#include <stdint.h>

/* STL Headers */
#include <string>

/* Project Headers */
#include "Debug.h"

/**
 * This class stores gps time of week and gps seconds and is used to provide a standard
 * method of storing this information between the Novatel and the Microstrain.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 16, 2012: Class creation
 */
class gps_time
{
public:
	enum TIME_STATUS
	{
		UNKNOWN = 20,
		APPROXIMATE = 60,
		COARSEADJUSTING = 80,
		COARSE = 100,
		COARSESTEERING = 120,
		FREEWHEELING = 130,
		FINEADJUSTING = 140,
		FINE = 160,
		FINESTEERING = 180,
		SATTIME = 200
	};
	/// initializes members to 0
	gps_time();
	/// initializeds week to week and scales milliseconds into seconds
	gps_time(uint16_t week, uint32_t milliseconds, TIME_STATUS status);
	/// initializes memebers to provided arguments
	gps_time(uint16_t week, double seconds, TIME_STATUS status);

	/// format the print the gps time
	friend Debug& operator<<(Debug& dbg, const gps_time& gt);

	/// assignment operator to copy gps_time
	gps_time& operator=(const gps_time& rhs);

	/// get the week
	inline uint16_t get_week() const {return week;}
	/// get the seconds
	inline double get_seconds() const {return seconds;}
	/// get the string version of the status
	std::string get_status_string() const;



private:
	/// GPS Week Number
	uint16_t week;
	/// GPS Time of Week scaled into seconds
	double seconds;
	/// GPS time status
	TIME_STATUS status;

};

Debug& operator<<(Debug& dbg, const gps_time& gt);

#endif
