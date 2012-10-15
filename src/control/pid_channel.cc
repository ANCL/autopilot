/**************************************************************************
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
 *************************************************************************/

#include "pid_channel.h"

pid_channel::pid_channel(double integrator_limit)
:_error(integrator_limit)
{

}

double pid_channel::compute_pid()
{
	return - gains().proportional() * error().proportional() - gains().derivative() * error().derivative() - gains().integral() * error().integral();
}

/* global functions */

Debug& operator<<(Debug& dbg, const pid_channel& ch)
{
	return dbg << "Channel " << ch.name() << ", error values: " << ch.error() << ", gain values: " << ch.gains();
}
