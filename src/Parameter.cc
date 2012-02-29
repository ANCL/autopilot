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

#include "Parameter.h"

Parameter::Parameter(std::string name, float value, int compid)
{
	setParamID(name);
	setValue(value);
	setCompID(compid);
}

void Parameter::setParamID(std::string name)
{
	if(name.empty())
		std::cout << "Empty parameter name received. Please enter a parameter name" << std::endl;

	int len = name.length();
	if(len > 14)
		// truncate string to 14 chars
		name = name.substr(0, 14);
	else if(len < 14)
	{
		// pad with spaces
		int strlen = 14 - len;
		name = name.append(strlen, ' ');
	}
	id = name;
}

Debug& operator<<(Debug& dbg, const Parameter& p)
{
	return dbg << "Parameter ID: " << p.getParamID() << ", Value: " << p.getValue();
}
