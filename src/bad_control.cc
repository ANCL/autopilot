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

#include "bad_control.h"

bad_control::bad_control(const std::string message, const std::string filename, const int line) throw()
{
	if (!filename.empty())
		this->message += filename + ' ';
	if (line != -1)
		this->message += boost::lexical_cast<std::string>(line) + ' ';
	this->message += message;
}

bad_control::bad_control(const bad_control& other) throw()
{
	message = other.message;
}

bad_control::~bad_control() throw()
{

}

bad_control& bad_control::operator=(const bad_control& other) throw()
{
	if (this == &other)
		return *this;
	message = other.message;

	return *this;
}
const char* bad_control::what() const throw()
{
  return message.c_str();
}

Debug& operator<<(Debug& dbg, const bad_control& bc)
{
	return dbg << bc.message;
}
