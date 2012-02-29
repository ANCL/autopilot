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

#ifndef INIT_FAILURE_H_
#define INIT_FAILURE_H_

#include <stdexcept>
#include <string>

class init_failure : public std::exception
{
public:
	init_failure(std::string message = std::string()) throw();
	init_failure(const init_failure& other) throw();
	~init_failure() throw();
	const char * what() const throw();
	init_failure& operator=(const init_failure& other) throw();
private:
	std::string message;
};
#endif
