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

#ifndef ATTITUDE_PID_H_
#define ATTITUDE_PID_H_

/* STL Headers */
#include <string>

/* Boost Headers */
#include <boost/thread.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/vector.hpp>
namespace blas = boost::numeric::ublas;

/* Project Headers */
#include "Parameter.h"
#include "pid_channel.h"
#include "ControllerInterface.h"

/**
 * @brief track pilot reference attitude
 * This class computes control signals using PID to track the pilot stick reference.
 * The angular velocity reference is assumed to be zero.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 26, 2011
 * @date February 10, 2012: Refactored into separate file and cleaned up
 */
class attitude_pid : public ControllerInterface
{
public:
	attitude_pid();
	attitude_pid(const attitude_pid& other);
	/**
	 * @brief Performs the control computation.  The control attempts to
	 * regulate the reference orientation with zero angular velocity.
	 * @param reference roll pitch reference values in radians.
	 */
	void operator()(const blas::vector<double>& reference) throw(bad_control);

	/// threadsafe get control_effort
	inline blas::vector<double> get_control_effort() const {boost::mutex::scoped_lock lock(control_effort_lock); return control_effort;}

	/// reset the integrator error states
	void reset();

	/**
	 * Return a list of parameters for transmission to QGC.  These parameters are only the ones
	 * specific to this controller.
	 */
	std::vector<Parameter> getParameters();
	/* Define some constants for parameter names */
	static const std::string PARAM_ROLL_KP;
	static const std::string PARAM_ROLL_KD;
	static const std::string PARAM_ROLL_KI;
	static const std::string PARAM_PITCH_KP;
	static const std::string PARAM_PITCH_KD;
	static const std::string PARAM_PITCH_KI;

	static const std::string PARAM_ROLL_TRIM;
	static const std::string PARAM_PITCH_TRIM;

	/** Set the roll channel proportional gain
	 * @param kp new proportional gain value
	 * This function is threadsafe.
	 */
	void set_roll_proportional(double kp);
	/** set the roll channel derivative gain
	 * @param kd new derivative gain value
	 * This function is threadsafe.
	 */
	void set_roll_derivative(double kd);
	/** set the roll channel integral gain
	 * @param ki new integral channel gain value
	 * This function is threadsafe.
	 */
	void set_roll_integral(double ki);
	/** Set the pitch channel proportional gain
	 * @param kp new proportional gain value
	 * This function is threadsafe.
	 */
	void set_pitch_proportional(double kp);
	/** Set the pitch channel derivative gain
	 * @param kd new derivative gain value
	 * This function is threadsafe.
	 */
	void set_pitch_derivative(double kd);
	/**
	 * Set the pitch channel integral gain
	 * @param ki new integral gain value
	 * This function is threadsafe.
	 */
	void set_pitch_integral(double ki);

	/**
	 * Outputs the controller parameters as an xml tree
	 * @param doc xml document in which to allocate the memory
	 * @returns pointer to xml node which contatins pid params
	 */
	rapidxml::xml_node<>* get_xml_node(rapidxml::xml_document<>& doc);
	/**
	 * Parses a pid parameter xml tree
	 * @param pid_params pointer to root of pid params xml tree
	 */
	void parse_pid(rapidxml::xml_node<> *pid_params);
	/** set the roll trim point.  threadsafe
	 * @param trim roll trim in radians
	 */
	inline void set_roll_trim_radians(double trim) {boost::mutex::scoped_lock lock(roll_trim_lock); roll_trim = trim;}
	/** get the roll trim point.  threadsafe
	 * @returns roll trim in radians
	 */
	inline double get_roll_trim_radians() {boost::mutex::scoped_lock lock(roll_trim_lock); return roll_trim;}
	/** set the roll trim point.  threadsafe
	 * @param trim roll trim in degrees
	 */
	void set_roll_trim_degrees(double trim_degrees);
	/** get the roll trim point.  threadsafe
	 * @returns roll trim in degress
	 */
	inline double get_roll_trim_degrees() {boost::mutex::scoped_lock lock(roll_trim_lock); return roll_trim * 180 / boost::math::constants::pi<double>();}
	/** set the pitch trim point.  threadsafe
	 * @param trim pitch trim in radians
	 */
	inline void set_pitch_trim_radians(double trim) {boost::mutex::scoped_lock lock(pitch_trim_lock); pitch_trim = trim;}
	/** get the pitch trim point.  threadsafe
	 * @returns pitch trim in radians
	 */
	inline double get_pitch_trim_radians() {boost::mutex::scoped_lock lock(pitch_trim_lock); return pitch_trim;}
	/** set the pitch trim point.  threadsafe
	 * @param trim pitch trim in degrees
	 */
	void set_pitch_trim_degrees(double trim_degrees);
	/** get the pitch trim point.  threadsafe
	 * @returns pitch trim in degress
	 */
	inline double get_pitch_trim_degrees() {boost::mutex::scoped_lock lock(pitch_trim_lock); return pitch_trim * 180 / boost::math::constants::pi<double>();}
	/// threadsafe get runnable
	inline bool runnable() const {return _runnable;}

private:
	pid_channel roll;
	mutable boost::mutex roll_lock;
	pid_channel pitch;
	mutable boost::mutex pitch_lock;

	/// store the current normalized servo commands
	blas::vector<double> control_effort;
	/// serialize access to control_effort
	mutable boost::mutex control_effort_lock;
	/// threadsafe set control_effort
	inline void set_control_effort(const blas::vector<double>& control_effort) {boost::mutex::scoped_lock lock(control_effort_lock); this->control_effort = control_effort;}

	/// trim point regulated by the controller.  it is stored in radians
	double roll_trim;
	/// serialize access to roll_trim
	boost::mutex roll_trim_lock;
	/// trim point regulated by the control.  stored in radians
	double pitch_trim;
	/// serialize access to pitch_trim
	boost::mutex pitch_trim_lock;

	/// is this controller runnable?
	bool _runnable;
	/// serialize access to runnable
	mutable boost::mutex runnable_lock;
	/// threadsafe clear _runnable
	inline void clear_runnable() {boost::mutex::scoped_lock lock(runnable_lock); _runnable = false;}
	/// threadsafe _runnable
	inline void set_runnable() {boost::mutex::scoped_lock lock(runnable_lock); _runnable = true;}


};
#endif
