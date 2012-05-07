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

#include "gps_time.h"

gps_time::gps_time()
: week(0), seconds(0)
{
}

gps_time::gps_time(uint16_t week, double seconds, TIME_STATUS status)
:week(week), seconds(seconds), status(status)
{
}

gps_time::gps_time(uint16_t week, uint32_t milliseconds, TIME_STATUS status)
:week(week), status(status)
{
	seconds = (static_cast<double>(milliseconds))/1000;
}

gps_time& gps_time::operator=(const gps_time& rhs)
{
	week = rhs.week;
	seconds = rhs.seconds;
	return *this;
}

std::string gps_time::get_status_string() const
{
	switch(status)
	{
	case UNKNOWN:
		return "Unknown";
	case APPROXIMATE:
		return "Approximate";
	case COARSEADJUSTING:
		return "Coarse Adjusting";
	case COARSESTEERING:
		return "Coarse Steering";
	case COARSE:
		return "Coarse";
	case FREEWHEELING:
		return "Freewheeling";
	case FINEADJUSTING:
		return "Fine Adjusting";
	case FINE:
		return "Fine";
	case FINESTEERING:
		return "Fine Steering";
	case SATTIME:
		return "Sat Time";
	default:
		return "not listed";
	}
	return std::string();
}

Debug& operator<<(Debug& dbg, const gps_time& gt)
{
	return dbg << "week number: " << gt.week << ", seconds: " << gt.seconds << " Status: " << gt.get_status_string();
}


