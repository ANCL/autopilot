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

#include "attitude_pid.h"

/* Project Headers */
#include "RCTrans.h"
#include "IMU.h"
#include "Control.h"

const std::string attitude_pid::PARAM_ROLL_KP = "PID_ROLL_KP";
const std::string attitude_pid::PARAM_ROLL_KD = "PID_ROLL_KD";
const std::string attitude_pid::PARAM_ROLL_KI = "PID_ROLL_KI";
const std::string attitude_pid::PARAM_PITCH_KP = "PID_PITCH_KP";
const std::string attitude_pid::PARAM_PITCH_KD = "PID_PITCH_KD";
const std::string attitude_pid::PARAM_PITCH_KI = "PID_PITCH_KI";
const std::string attitude_pid::PARAM_ROLL_TRIM = "TRIM_ROLL";
const std::string attitude_pid::PARAM_PITCH_TRIM = "TRIM_PITCH";

attitude_pid::attitude_pid()
:roll(5),
 pitch(5),
 control_effort(blas::zero_vector<double>(2)),
 roll_trim(0),
 pitch_trim(0),
 _runnable(true)
{
	roll.name() = "Roll";
	pitch.name() = "Pitch";

	LogFile *log = LogFile::getInstance();
	log->logHeader(heli::LOG_ATTITUDE_ERROR, "Roll_Proportional Roll_Derivative Roll_Integral Pitch_Proportional Pitch_Derivative Pitch_Integral");
	log->logData(heli::LOG_ATTITUDE_ERROR, std::vector<double>());
	log->logHeader(heli::LOG_ATTITUDE_REFERENCE, "Roll Pitch");
	log->logData(heli::LOG_ATTITUDE_REFERENCE, std::vector<double>());

}

attitude_pid::attitude_pid(const attitude_pid& other)
{

	{
		boost::mutex::scoped_lock lock(other.control_effort_lock);
		control_effort = other.control_effort;
	}
	{
		boost::mutex::scoped_lock lock(other.roll_lock);
		roll = other.roll;
	}
	{
		boost::mutex::scoped_lock lock(other.pitch_lock);
		pitch = other.pitch;
	}
	{
		boost::mutex::scoped_lock lock(other.runnable_lock);
		_runnable = other._runnable;
	}
}

void attitude_pid::reset()
{
	roll.reset();
	pitch.reset();
}

void attitude_pid::operator()(const blas::vector<double>& reference) throw(bad_control)
{
	if (!runnable())
		throw bad_control("attempted to compute attitude_pid, but it wasn't runnable.");

	if (reference.size() < 2)
		throw bad_control("Attitude control received less than two references (roll pitch)");

	blas::vector<double> roll_pitch_reference(reference);
	roll_pitch_reference.resize(2);

	IMU* imu = IMU::getInstance();
	blas::vector<double> euler(imu->get_euler());
	blas::vector<double> euler_rate(imu->get_euler_rate());
	// resize to perform vector subtraction
	euler.resize(2);


	blas::vector<double> euler_error(euler - roll_pitch_reference);

	std::vector<double> log(euler_error.begin(), euler_error.end());
	log.insert(log.end(), euler_rate.begin(), euler_rate.end()-1);
	LogFile::getInstance()->logData("Attitude PID error", log);
	blas::vector<double> control_effort(2);
	control_effort.clear();

	std::vector<double> error_states;
	roll_lock.lock();
	error_states.push_back(roll.error().proportional() = euler_error[0]);
	error_states.push_back(roll.error().derivative() = euler_rate[0]);
	error_states.push_back(++roll.error());
	control_effort[0] = roll.compute_pid();
	roll_lock.unlock();

	pitch_lock.lock();
	error_states.push_back(pitch.error().proportional() = euler_error[1]);
	error_states.push_back(pitch.error().derivative() = euler_rate[1]);
	error_states.push_back(++pitch.error());

	LogFile::getInstance()->logData(heli::LOG_ATTITUDE_ERROR, error_states);
	control_effort[1] = pitch.compute_pid();
	pitch_lock.unlock();

	// saturate the controls to [-1, 1]
	Control::saturate(control_effort);
	set_control_effort(control_effort);

	LogFile::getInstance()->logData(heli::LOG_ATTITUDE_CONTROL_EFFORT, control_effort);
//	debug() << "Attitude PID control effort: " << control_effort;
}

std::vector<Parameter> attitude_pid::getParameters()
{
//	boost::mutex::scoped_lock lock(read_params_lock);
	std::vector<Parameter> plist;

	roll_lock.lock();
	plist.push_back(Parameter(PARAM_ROLL_KP, roll.gains().proportional(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_ROLL_KD, roll.gains().derivative(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_ROLL_KI, roll.gains().integral(), heli::CONTROLLER_ID));
	roll_lock.unlock();

	pitch_lock.lock();
	plist.push_back(Parameter(PARAM_PITCH_KP, pitch.gains().proportional(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_PITCH_KD, pitch.gains().derivative(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_PITCH_KI, pitch.gains().integral(), heli::CONTROLLER_ID));
	pitch_lock.unlock();

	plist.push_back(Parameter(PARAM_ROLL_TRIM, get_roll_trim_degrees(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_PITCH_TRIM, get_pitch_trim_degrees(), heli::CONTROLLER_ID));

	return plist;
}

void attitude_pid::set_roll_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(roll_lock);
		roll.gains().proportional() = kp;
	}
	message() << "Set roll proportional gain to: " << kp;
}

void attitude_pid::set_roll_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(roll_lock);
		roll.gains().derivative() = kd;
	}
	message() << "Set roll derivative gain to: " << kd;
}

void attitude_pid::set_roll_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(roll_lock);
		roll.gains().integral() = ki;
	}
	message() << "Set roll integral gain to: " << ki;
}

void attitude_pid::set_pitch_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(pitch_lock);
		pitch.gains().proportional() = kp;
	}
	message() << "Set pitch proportional gain to: " << kp;
}

void attitude_pid::set_pitch_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(pitch_lock);
		pitch.gains().derivative() = kd;
	}
	message() << "Set pitch derivative gain to: " << kd;
}

void attitude_pid::set_pitch_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(pitch_lock);
		pitch.gains().integral() = ki;
	}
	message() << "Set pitch integral gain to: " << ki;
}
void attitude_pid::set_roll_trim_degrees(double trim_degrees)
{
	{
		boost::mutex::scoped_lock lock(roll_trim_lock);
		roll_trim = trim_degrees * boost::math::constants::pi<double>()/180;
	}
	message() << "Set roll trim to " << trim_degrees << " deg.";
}

void attitude_pid::set_pitch_trim_degrees(double trim_degrees)
{
	{
		boost::mutex::scoped_lock lock(pitch_trim_lock);
		pitch_trim = trim_degrees * boost::math::constants::pi<double>()/180;
	}
	message() << "Set pitch trim to " << trim_degrees << " deg.";
}


rapidxml::xml_node<>* attitude_pid::get_xml_node(rapidxml::xml_document<>& doc)
{
	rapidxml::xml_node<> *pid_node = doc.allocate_node(rapidxml::node_element, "attitude_pid");

	{
		//roll
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		rapidxml::xml_node<> *channel_node = doc.allocate_node(rapidxml::node_element, "channel");
		attr = doc.allocate_attribute("name", "roll");
		channel_node->append_attribute(attr);
		pid_node->append_node(channel_node);

		// roll proportional
		roll_lock.lock();
		node_value = doc.allocate_string(boost::lexical_cast<std::string>(roll.gains().proportional()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
		attr = doc.allocate_attribute("type", "proportional");
		node->append_attribute(attr);
		channel_node->append_node(node);

		// roll derivative
		node_value = doc.allocate_string(boost::lexical_cast<std::string>(roll.gains().derivative()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
		attr = doc.allocate_attribute("type", "derivative");
		node->append_attribute(attr);
		channel_node->append_node(node);

		// roll integral
		node_value = doc.allocate_string(boost::lexical_cast<std::string>(roll.gains().integral()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
		attr = doc.allocate_attribute("type", "integral");
		node->append_attribute(attr);
		roll_lock.unlock();
		channel_node->append_node(node);
	}

	{
		//pitch
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		rapidxml::xml_node<> *channel_node = doc.allocate_node(rapidxml::node_element, "channel");
		attr = doc.allocate_attribute("name", "pitch");
		channel_node->append_attribute(attr);
		pid_node->append_node(channel_node);

		// pitch proportional
		pitch_lock.lock();
		node_value = doc.allocate_string(boost::lexical_cast<std::string>(pitch.gains().proportional()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
		attr = doc.allocate_attribute("type", "proportional");
		node->append_attribute(attr);
		channel_node->append_node(node);

		// pitch derivative
		node_value = doc.allocate_string(boost::lexical_cast<std::string>(pitch.gains().derivative()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
		attr = doc.allocate_attribute("type", "derivative");
		node->append_attribute(attr);
		channel_node->append_node(node);

		// pitch integral
		node_value = doc.allocate_string(boost::lexical_cast<std::string>(pitch.gains().integral()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
		attr = doc.allocate_attribute("type", "integral");
		node->append_attribute(attr);
		pitch_lock.unlock();
		channel_node->append_node(node);
	}



	{
		// roll trim
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_roll_trim_degrees()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "roll_trim", node_value);
		pid_node->append_node(node);
	}

	{
		// pitch trim
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_pitch_trim_degrees()).c_str());
		node = doc.allocate_node(rapidxml::node_element, "pitch_trim", node_value);
		pid_node->append_node(node);
	}

	return pid_node;
}

void attitude_pid::parse_pid(rapidxml::xml_node<> *pid_params)
{
	for (rapidxml::xml_node<> *channel = pid_params->first_node(); channel; channel = channel->next_sibling())
	{
		if (boost::to_upper_copy(std::string(channel->name())) == "CHANNEL")
		{
			rapidxml::xml_attribute<> *attr;
			for (attr = channel->first_attribute(); attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
			std::string channel_name(attr->value());
			boost::to_upper(channel_name);

			for (rapidxml::xml_node<> *gain = channel->first_node(); gain; gain = gain->next_sibling())
			{
				// get value
				std::string gain_value(gain->value());
				boost::trim(gain_value);

				// find which gain it is
				rapidxml::xml_attribute<> *attr;
				for (attr = gain->first_attribute(); attr && std::string(attr->name()) != "type"; attr = attr->next_attribute());
				std::string gain(attr->value());
				boost::to_upper(gain);

				if (channel_name == "ROLL")
				{
					if (gain == "PROPORTIONAL")
						set_roll_proportional(boost::lexical_cast<double>(gain_value));
					else if (gain == "DERIVATIVE")
						set_roll_derivative(boost::lexical_cast<double>(gain_value));
					else if (gain == "INTEGRAL")
						set_roll_integral(boost::lexical_cast<double>(gain_value));
					else
						warning() << "parse_pid(): Unknown gain on roll channel: " << gain;
				}
				else if (channel_name == "PITCH")
				{
					if (gain == "PROPORTIONAL")
						set_pitch_proportional(boost::lexical_cast<double>(gain_value));
					else if (gain == "DERIVATIVE")
						set_pitch_derivative(boost::lexical_cast<double>(gain_value));
					else if (gain == "INTEGRAL")
						set_pitch_integral(boost::lexical_cast<double>(gain_value));
					else
						warning() << "parse_pid(): Unknown gain on pitch channel: " << gain;
				}
				else
					warning() << "parse_pid(): Unknown channel: " << channel_name;
			}
		}

		else if (boost::to_upper_copy(std::string(channel->name())) == "ROLL_TRIM")
		{
			std::string roll_trim_value(channel->value());
			boost::trim(roll_trim_value);
			set_roll_trim_degrees(boost::lexical_cast<double>(roll_trim_value));
		}
		else if (boost::to_upper_copy(std::string(channel->name())) == "PITCH_TRIM")
		{
			std::string pitch_trim_value(channel->value());
			boost::trim(pitch_trim_value);
			set_pitch_trim_degrees(boost::lexical_cast<double>(pitch_trim_value));
		}
		else
			warning() << "parse_pid(): unknown xml node " << channel->name();
	}
}
