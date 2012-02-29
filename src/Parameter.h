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

#ifndef PARAMETER_H_
#define PARAMETER_H_

/* STL Headers */
#include <iostream> // used only while quick debugging (to print to screen).
#include <string>
#include <vector>

/* Boost Headers */
#include <boost/algorithm/string.hpp>

/* Project Headers */
#include "Debug.h"

/**
 * @brief Contains a parameter for sending to QGroundControl.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date July 25, 2011: Class creation
 * @date November 9, 2011: Added debug operator<<
 */
class Parameter
{
public:
	explicit Parameter(std::string name = std::string(), float value = 0.0, int compid = 0);
	void setParamID(std::string name);
	void setValue(float value){this->value = value;}
	void setCompID(int value){this->component_id = value;}

	std::string getParamID() const {return id;}
	std::string getParamID_trimmed() const {return boost::trim_copy(id);}
	float getValue() const {return value;}
	int getCompID() const {return component_id;}

	friend Debug& operator<<(Debug& dbg, const Parameter& p);

private:

	std::string id;
	float value;
	int component_id;

};
#endif // PARAMETER_H_
