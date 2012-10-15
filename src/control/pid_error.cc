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

#include "pid_error.h"

/* c headers */
#include <math.h>


pid_error::pid_error(double integral_error_limit)
:_integral_error_limit(integral_error_limit)
{
	error.assign(0);
}

double& pid_error::operator++()
{
	integral() += proportional()*0.01; // multiply by the timestep
	if(fabs(integral()) > _integral_error_limit)
		integral() = 0;
	return integral();
}


/* Global functions */
std::ostream& operator<<(std::ostream& os, const pid_error& error)
{
	for (boost::array<double, 3>::const_iterator it=error.error.begin(); it != error.error.end()-1; ++it)
		os << *it << ", ";
	return os << error.error.back();
}

Debug& operator<<(Debug& dbg, const pid_error& error)
{
	for (boost::array<double, 3>::const_iterator it=error.error.begin(); it != error.error.end()-1; ++it)
		dbg << *it << ", ";
	return dbg << error.error.back();
}
