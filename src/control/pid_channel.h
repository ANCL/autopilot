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

#ifndef PID_CHANNEL_H_
#define PID_CHANNEL_H_

/* Project Headers */
#include "pid_gains.h"
#include "pid_error.h"
#include "Debug.h"

/**
 * @brief this class contains all the relevant information for a channel
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 27, 2011: Class creation
 * @date February 10, 2012: Refactor out of Control
 * @date October 15, 2012: Added constructor to allow setting integrator reset limit
 */
class pid_channel
{
public:
	pid_channel(double integrator_limit = 1);
	/**
	 * @returns gains as lvalue
	 */
	pid_gains& gains() {return _gains;}
	/**
	 * @returns gains as rvalue
	 */
	const pid_gains& gains() const {return _gains;}
	/**
	 * @returns error object as lvalue
	 */
	pid_error& error() {return _error;}
	/**
	 * @returns error object as rvalue
	 */
	const pid_error& error() const {return _error;}
	/**
	 * @returns name of channel as lvalue
	 */
	std::string& name() {return _name;}
	/**
	 * @returns name of channel as rvalue
	 */
	const std::string& name() const {return _name;}
	/**
	 * stream insertion operator for outputing the state of the object
	 */
	friend std::ostream& operator<<(std::ostream& os, const pid_channel& ch);
	/**
	 * stream insertion for debugging object
	 */
	friend Debug& operator<<(Debug& dbg, const pid_channel& ch);

	void reset() {_error.reset();}

	/**
	 * Perform actual PID computation
	 * @returns computed control effort for this channel
	 */
	double compute_pid();
private:
	/// Store the gains for the channels being controlled
	pid_gains _gains;
	/// Store the errors for the channels being controlled
	pid_error _error;
	/// Store the name of the channel
	std::string _name;
};

Debug& operator<<(Debug& dbg, const pid_channel& ch);

#endif
