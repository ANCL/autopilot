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

#ifndef PID_GAINS_H_
#define PID_GAINS_H_

/* Boost Headers */
#include <boost/array.hpp>

/* Project Headers */
#include "Debug.h"

/**
 * @brief Store the gains for PID control
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 27, 2011: Class creation
 * @date February 10, 2012: Refactored out of class control
 */
class pid_gains
{
public:
	/**
	 * Initializes the gains to 1
	 */
	pid_gains() {gains.assign(1);}
	/** copy constructor */
	pid_gains(const pid_gains& other) {gains = other.gains;}
	/** proportional gain as lvalue */
	double& proportional() {return gains[0];}
	/** proportional gain as rvalue */
	const double& proportional() const {return gains[0];}
	double& derivative() {return gains[1];}
	const double& derivative() const {return gains[1];}
	double& integral() {return gains[2];}
	const double& integral() const {return gains[2];}
	/**
	 * Stream insertion for Debug object
	 */
	friend Debug& operator<<(Debug& dbg, const pid_gains& gains);
private:
	boost::array<double, 3> gains;
};

Debug& operator<<(Debug& dbg, const pid_gains& gains);
#endif
