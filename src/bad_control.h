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

#ifndef BAD_CONTROL_H_
#define BAD_CONTROL_H_

/* STL Headers */
#include <stdexcept>
#include <string>

/* Boost Headers */
#include <boost/lexical_cast.hpp>

/* Project Headers */
#include "Debug.h"

class bad_control : public std::exception
{
public:
	explicit bad_control(const std::string message = std::string(), const std::string filename = std::string(), const int line = -1) throw();
	bad_control(const bad_control& other) throw();
	~bad_control() throw();
	bad_control& operator= (const bad_control& other) throw();
	const char * what() const throw();

	friend Debug& operator<<(Debug& dbg, const bad_control& bc);

private:
	std::string message;

};

Debug& operator<<(Debug& dbg, const bad_control& bc);

#endif /* BAD_CONTROL_H_ */
