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

#ifndef CONTROL_H_
#define CONTROL_H_
/* STL Headers */
#include <vector>
#include <string>

/* Project Headers */
#include "Parameter.h"
#include "attitude_pid.h"
#include "translation_outer_pid.h"
#include "ControllerInterface.h"
#include "tail_sbf.h"
#include "IMU.h"

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
namespace blas = boost::numeric::ublas;

/**
 * @brief Perform automatic control computation
 *
 * This class contains all the control code used for the autopilot.
 * Each control method should be executed as the functor of a nested class.
 * This design allows the variables used for each control algorithm to be separated,
 * which should result in a modular and easily expandable architecture.
 *
 * The member functions of this class implement features which are common to all control methods.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 * @date October 26, 2011: Class creation
 * @date Januray 12, 2012: Added translational PID control
 * @date February 10, 2012: Refactored to comply with ControllerInterface
 * @date September 27, 2012: Added Tail SBF Controller
 */
class Control : public ControllerInterface
{
public:

	static Control* getInstance();
	/**
	 * Creates a list of all control related parameters.  In other words the list will include
	 * the parameters stored in the class (which are general to all controls e.g., pilot mix) as well
	 * as all the parameters for the different types of controls.
	 * @returns Parameter list to send to QGC
	 */
	std::vector<Parameter> getParameters();
	/**
	 * Sets the value of a parameter.  Usually called when a message is received from QGC.
	 * @param p parameter to change
	 * @note make sure that the param_id field has been trimmed to remove white space inserted by QGC
	 */
	void setParameter(Parameter p);
	/**
	 *
	 * @returns the (normalized) values to be sent to the helicopter.  These values
	 * have been mixed with the pilot inputs using the weights stored in
	 * Control::pilot_mix.
	 */
	blas::vector<double> get_control_effort() const;
	/**
	 * This function computes the control effort for the autopilot.  How the control is computed
	 * depends on which mode the controller is operating in.  If Control::controller_mode is
	 * heli::MODE_ATTITUDE_STABILIZATION_PID then only the roll-pitch are regulated by using the pilot
	 * input as a reference (see Control::augmented_stability_pid).  When Control::controller_mode is heli::MODE_POSITION_HOLD_PID, the roll-pitch
	 * reference is generated by an outer translational pid controller (see Control::translation_outer_pid),
	 * the same attitude controller is then used as for heli::MODE_ATTITUDE_STABILIZATION_PID.  In this mode,
	 * the gps measurement is required to be valid (GPS::pos_is_valid and GPS::vel_is_valid).
	 *
	 */
	void operator()();

	/// only declared to be compatible with ControllerInterface
	void operator()(const blas::vector<double>& reference) throw(bad_control){}

	/// string representation of roll mix parameter
	static const std::string PARAM_MIX_ROLL;
	/// string representation of pitch mix parameter
	static const std::string PARAM_MIX_PITCH;
	/// string representation of controller mode parameter
	static const std::string CONTROL_MODE;

	/// @returns a reference to the pid contoller used for roll-pitch
	attitude_pid& attitude_pid_controller() {return roll_pitch_pid_controller;}

	const attitude_pid& attitude_pid_controller() const {return roll_pitch_pid_controller;}

	/// @returns a ref to the pid controller for translation
	translation_outer_pid& translation_pid_controller() {return x_y_pid_controller;}

	const translation_outer_pid& translation_pid_controller() const {return x_y_pid_controller;}


	/// return a string representation of mode
	static std::string getModeString(heli::Controller_Mode mode);

	/// emitted when the controller mode changes
	boost::signals2::signal<void (heli::Controller_Mode)> mode_changed;

	void reset();

	bool runnable() const;

	/// saturate all the elements of a vector such that -bound <= signal[i] <= bound
	template <typename ContainerType>
	static void saturate(ContainerType& signal, double bound = 1);
	/// same as Control::saturate but also return a copy
	template <typename ContainerType>
	static ContainerType saturate_copy(const ContainerType& signal, double bound = 1);

	/// set the reference position from the imu when the pilot mode changes to autopilot
	void set_reference_position();
	/// threadsafe set controller mode
	void set_controller_mode(heli::Controller_Mode mode);
	/// threadsafe get controller mode
	inline heli::Controller_Mode get_controller_mode() const {boost::mutex::scoped_lock lock(controller_mode_lock); return controller_mode;}

	/// threadsafe access reference_position
	blas::vector<double> get_reference_position() const {boost::mutex::scoped_lock lock(reference_position_lock); return reference_position;}

	/// return the difference between the current position and the reference position in the body frame
	blas::vector<double> get_body_postion_error() const {return prod(trans(IMU::getInstance()->get_rotation()), get_ned_position_error());}

	/// return the position error in the navigation frame
	blas::vector<double> get_ned_position_error() const {return IMU::getInstance()->get_ned_position() - get_reference_position();}

	/// return the current reference attitude
	blas::vector<double> get_reference_attitude() const {boost::mutex::scoped_lock(reference_attitude_lock); return reference_attitude;}

private:
	Control();

	enum channel_names
	{
		ROLL = 0,
		PITCH = 1,
		CONTROLLED_CHANNELS
	};
	/// Pointer to the instance of this class
	static Control* _instance;
	/// Make access to Control::_instance threadsafe
	static boost::mutex _instance_lock;

	/// PID Controller object to control the inner loop roll-pitch
	attitude_pid roll_pitch_pid_controller;

	/** PID controller to stabilized a position.  It provides a roll-pitch reference
	 * for the inner controller
	 */
	translation_outer_pid x_y_pid_controller;

	/// PID control with tail rotor sbf compensation
	tail_sbf x_y_sbf_controller;

	/**
	 * Stores the weighting used to calculate the pilot mix with the controller output.
	 * These values multiply the pilot inputs (1 - pilot_mix) multiplies the control.
	 */
	std::vector<double> pilot_mix;
	/// Serializes access to pilot_mix
	mutable boost::mutex pilot_mix_lock;

	/**
	 * Set the weight of the pilot input on the roll channel.
	 * This function is threadsafe.
	 */
	void set_roll_mix(double roll_mix);

	/**
	 * Set the weight of the pilot input on the pitch channel.
	 * This function is threadsafe.
	 */
	void set_pitch_mix(double pitch_mix);

	/**
	 * Stores the contents of the xml configuration file
	 */
	char *config_file_buffer;

	/**
	 * Strores xml info for config file
	 */
	rapidxml::xml_document<> config_file_xml;

	/**
	 * Serializes access to the controller paramter file
	 */
	mutable boost::mutex config_file_lock;

	/**
	 * Reads the configuration file stored in heli::controller_param_filename and
	 * populates config_file_buffer, and config_file_xml.  Any parameter values
	 * from the file are loaded into the appropriate date members.
	 */
	void loadFile();
	/**
	 * Parses a pilot mix xml node and applies the appropriate settings
	 * @param mix pointer to pilot mix xml node
	 */
	void parse_pilot_mix(rapidxml::xml_node<> *mix);
	/**
	 * Parse controller mode xml node and set the corresponding mode
	 */
	void parse_mode(rapidxml::xml_node<> *mode);
	/**
	 * save the state of the controller in an xml file
	 */
	void saveFile();

	/// store the current controller mode
	heli::Controller_Mode controller_mode;
	/// serialize access to controller_mode
	mutable boost::mutex controller_mode_lock;

	/// connection to qgc mode signal
	boost::signals2::scoped_connection mode_connection;

	/// ned reference position used for translation control
	blas::vector<double> reference_position;
	/// serialize access to reference_position
	mutable boost::mutex reference_position_lock;

	/// reference attitude (roll-pitch) either generated by outer loop or attitude trim
	blas::vector<double> reference_attitude;
	/// serialize access to reference_attitude
	mutable boost::mutex reference_attitude_lock;
	/// set the reference attitude
	void set_reference_attitude(const blas::vector<double>& reference_attitude) {boost::mutex::scoped_lock(reference_attitude_lock); this->reference_attitude = reference_attitude;}


	/// threadsafe set reference_position
	void set_reference_position(const blas::vector<double>& position) {boost::mutex::scoped_lock lock(reference_position_lock); reference_position = position;}

};

template <typename ContainerType>
void Control::saturate(ContainerType& signal, double bound)
{
	for (typename ContainerType::iterator it = signal.begin(); it != signal.end(); ++it)
	{
		if (*it > bound)
			*it = bound;
		if (*it < -bound)
			*it = -bound;
	}
}

template <typename ContainerType>
ContainerType Control::saturate_copy(const ContainerType& signal, double bound)
{
	ContainerType signal_copy(signal);
	saturate(signal_copy, bound);
	return signal_copy;
}

#endif /* CONTROL_H_ */
