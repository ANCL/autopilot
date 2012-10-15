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

#ifndef PID_ERROR_H_
#define PID_ERROR_H_

/* Boost Headers */
#include <boost/array.hpp>

/* STL Headers */
#include <iostream>

/* Project Headers */
#include "Debug.h"

/**
 * @brief Store the error for PID control. (Integral error reset for all two channels if its magnitude exceeds a limit)
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 27, 2011: Class creation
 * @date February 10, 2012: Refactored out of class Control
 */
class pid_error
{
public:
	/**
	 * Initializes error to all zeros
	 */
	pid_error(double integral_error_limit = 1);
	/**
	 * @returns proportional error as lvalue
	 */
	double& proportional() {return error[0];}
	const double& proportional() const {return error[0];}
	double& derivative() {return error[1];}
	const double& derivative() const {return error[1];}
	double& integral() {return error[2];}
	const double& integral() const {return error[2];}
	/**
	 * Easy way to increment integral error
	 * When the integral is computed, if it's absolute value is greater than GPS::Error::_integral_error_limit
	 * the integral state is reset.
	 * @returns result integral error state state
	 */
	double& operator++();

	/**
	 * Stream insertion for std::ostream (cerr, cout)
	 */
	friend std::ostream& operator<<(std::ostream& os, const pid_error& error);
	/**
	 * Stream insertion operator for Debug object
	 */
	friend Debug& operator<<(Debug& dbg, const pid_error& error);
	/// zero all the errors
	void reset() {error.assign(0);}
private:
	boost::array<double, 3> error;
	double _integral_error_limit;
};

Debug& operator<<(Debug& dbg, const pid_error& error);
#endif
