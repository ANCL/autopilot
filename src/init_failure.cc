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

#include "init_failure.h"

init_failure::init_failure(std::string message) throw()
:message(message)
{

}

init_failure::init_failure(const init_failure& other) throw()
{
	message = other.message;
}

init_failure::~init_failure() throw()
{

}

const char * init_failure::what() const throw()
{
	return message.c_str();
}

init_failure& init_failure::operator=(const init_failure& other) throw()
{
	if (this == &other)
		return *this;
	message = other.message;
	return *this;
}
