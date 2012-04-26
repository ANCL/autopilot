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

gps_time::gps_time(uint16_t week, double seconds)
:week(week), seconds(seconds)
{
}

gps_time::gps_time(uint16_t week, uint32_t milliseconds)
:week(week)
{
	seconds = (static_cast<double>(milliseconds))/1000;
}

Debug& operator<<(Debug& dbg, const gps_time& gt)
{
	return dbg << "Time of week: " << gt.week << ", seconds: " << gt.seconds;
}
