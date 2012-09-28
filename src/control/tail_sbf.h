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

/**
 * @brief Implement translation control with tail rotor sbf compensation as described in ACC2013 Godbolt/Lynch
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date: September 26, 2012: Class creation
 */
#ifndef TAIL_SBF_H_
#define TAIL_SBF_H_

/* Boost Headers */
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/vector.hpp>
namespace blas = boost::numeric::ublas;

/* Project Headers */
#include "pid_channel.h"
#include "ControllerInterface.h"
#include "Parameter.h"

/* STL Headers */
#include <vector>
#include <string>

class tail_sbf : public ControllerInterface
{
public:
	tail_sbf();

	/// reset the integrator states
	void reset();

	/// test if the controller is safe to run ** NOT IMPLEMENTED **
	bool runnable() const;

	/// integrate position error, and compute the resulting control effort
	void operator()(const blas::vector<double>& reference) throw(bad_control);

	/// @returns the roll pitch reference in radians (threadsafe)
	inline blas::vector<double> get_control_effort() const {boost::mutex::scoped_lock lock(control_effort_lock); return control_effort;}

	/// return the scaled travel in degrees
	inline double scaled_travel_degrees() const {boost::mutex::scoped_lock lock(scaled_travel_lock); return scaled_travel;}
	/// return the scaled travel in radians
	inline double scaled_travel_radians() const {return scaled_travel_degrees()*boost::math::constants::pi<double>()/180;}
	/// set the scaled travel in degrees
	inline void set_scaled_travel_degrees(double travel) {set_scaled_travel(travel);}
	/// set the scaled travel in radians
	inline void set_scaled_travel_radians(double travel) {set_scaled_travel(travel*boost::math::constants::pi<double>()/180);}

	/// get the parameter list
	std::vector<Parameter> getParameters() const;

	static const std::string PARAM_X_KP;
	static const std::string PARAM_X_KD;
	static const std::string PARAM_X_KI;

	static const std::string PARAM_Y_KP;
	static const std::string PARAM_Y_KD;
	static const std::string PARAM_Y_KI;

	static const std::string PARAM_TRAVEL;
private:
	/// error states in ned x,y directions
	pid_channel ned_x, ned_y;
	/// serialize access to error states
	mutable boost::mutex ned_x_lock, ned_y_lock;

	/// store the current control effort
	blas::vector<double> control_effort;
	/// serialize access to control_effort
	mutable boost::mutex control_effort_lock;
	/// threadsafe set control_effort
	inline void set_control_effort(const blas::vector<double>& control_effort) {boost::mutex::scoped_lock lock(control_effort_lock); this->control_effort = control_effort;}

	/**
	 * Stores the rotational travel which is used to map the output of the
	 * translational control into an angle reference.  This value is stored in degrees.
	 * @note Roll and pitch use the same value.
	 */
	double scaled_travel;
	/// serializes access to scaled_travel
	mutable boost::mutex scaled_travel_lock;
	/**
	 * Set the scaled travel used to represent the pilot stick position
	 * @param travel new travel value in degrees
	 * This function is threadsafe.
	 */
	void set_scaled_travel(double travel);
};

#endif /* TAIL_SBF_H_ */
