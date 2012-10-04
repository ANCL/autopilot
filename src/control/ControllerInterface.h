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

#ifndef CONTROLLERINTERFACE_H_
#define CONTROLLERINTERFACE_H_

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
namespace blas = boost::numeric::ublas;

/* Project Headers */
#include "bad_control.h"

/**
 * @brief defines a standard interface for a controller class.
 * This class declares functions which should be present in all controllers.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 10, 2012: Class creation
 */
class ControllerInterface
{
public:
	/**
	 * @brief Called when the control is switched in.
	 *	This function should reset any controller states (e.g., error states).
	 */
	virtual void reset() = 0;
	/**
	 * Returns true if the controller can be executed.
	 * This function is used to tell if a condition exists which prevents
	 * this controller from working properly (e.g., gps outage for position control).
	 */
	virtual bool runnable() const = 0;
	/**
	 * Compute the control and return the control effort.  The difference between this
	 * function and ControllerInterface::get_control_effort is that any controller states
	 * are updated (i.e., integrated).  This function should only be called once per timestep.
	 * If an error unrecoverable condition is detected during the control computation which prevents
	 * a valid control effort from being computed a bad_control object will be thrown.
	 */
	virtual void operator()(const blas::vector<double>& reference) throw(bad_control) = 0;
	/**
	 * Return the control effort.  This function does not actually compute the control
	 * (i.e., integrate any states) since it may be called several times per timestep.
	 * @returns current control effort
	 */
	virtual blas::vector<double> get_control_effort() const = 0;
};
#endif
