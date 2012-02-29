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

#ifndef TRANSLATION_OUTER_PID_H_
#define TRANSLATION_OUTER_PID_H_

/* STL Headers */
#include <vector>
#include <string>

/* Project Headers */
#include "Parameter.h"
#include "pid_channel.h"
#include "GPS_Filter.h"
#include "ControllerInterface.h"

/* Boost Headers */
#include <boost/array.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/signals2/signal.hpp>

/**
 * @brief outer loop pid controller for body frame x-y control
 * This controller is outer loop in the sense that it provides
 * an attitude reference (roll pitch) which must be regulated by
 * an inner loop attitude controller
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 * @date January 2012: Class Creation
 * @date February 10, 2012: Refactored into separate file
 */
class translation_outer_pid : public ControllerInterface
{
public:
	translation_outer_pid();
	translation_outer_pid(const translation_outer_pid& other);
	/** Set the x channel proportional gain
	 * @param kp new proportional gain value
	 * This function is threadsafe.
	 */
	void set_x_proportional(double kp);
	/** set the x channel derivative gain
	 * @param kd new derivative gain value
	 * This function is threadsafe.
	 */
	void set_x_derivative(double kd);
	/** set the x channel integral gain
	 * @param ki new integral channel gain value
	 * This function is threadsafe.
	 */
	void set_x_integral(double ki);
	/** Set the y channel proportional gain
	 * @param kp new proportional gain value
	 * This function is threadsafe.
	 */
	void set_y_proportional(double kp);
	/** Set the y channel derivative gain
	 * @param kd new derivative gain value
	 * This function is threadsafe.
	 */
	void set_y_derivative(double kd);
	/**
	 * Set the y channel integral gain
	 * @param ki new integral gain value
	 * This function is threadsafe.
	 */
	void set_y_integral(double ki);
	/**
	 * Perform the pid control computation and return
	 * the roll pitch reference
	 */
	void operator()(const blas::vector<double>& reference) throw(bad_control);
	/// @returns the roll pitch reference in radians (threadsafe)
	inline blas::vector<double> get_control_effort() const {boost::mutex::scoped_lock lock(control_effort_lock); return control_effort;}

	/// @returns the list of parameters for the translational pid outer loop
	std::vector<Parameter> getParameters();

	/// x kp parameter representation
	static const std::string PARAM_X_KP;
	/// x kd parameter string representation
	static const std::string PARAM_X_KD;
	/// x ki parameter string representation
	static const std::string PARAM_X_KI;
	/// y kp parameter string representation
	static const std::string PARAM_Y_KP;
	/// y kd parameter string representation
	static const std::string PARAM_Y_KD;
	/// y ki parameter string representation
	static const std::string PARAM_Y_KI;
	static const std::string PARAM_TRAVEL;
	/// create and xml tree with the controller parameters
	rapidxml::xml_node<>* get_xml_node(rapidxml::xml_document<>& doc);
	/// parse an xml tree containing the parameters for the function and populate the values
	void parse_xml_node(rapidxml::xml_node<> *pid_params);
	/// resets the controller
	void reset();
	/// test is controller is runnable
	bool runnable() const;

	inline double scaled_travel_degrees() {boost::mutex::scoped_lock lock(scaled_travel_lock); return scaled_travel;}
	inline double scaled_travel_radians() {boost::mutex::scoped_lock lock(scaled_travel_lock); return scaled_travel*boost::math::constants::pi<double>()/180;}
	inline void set_scaled_travel_degrees(double travel) {set_scaled_travel(travel);}
	inline void set_scaled_travel_radians(double travel) {set_scaled_travel(travel*boost::math::constants::pi<double>()/180);}
private:
	pid_channel x;
	mutable boost::mutex x_lock;
	pid_channel y;
	mutable boost::mutex y_lock;

	boost::array<GPS_Filter, 3> pos_filters;
	boost::array<GPS_Filter, 3> vel_filters;

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
#endif
