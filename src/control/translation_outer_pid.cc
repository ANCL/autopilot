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

#include "translation_outer_pid.h"

/* Project Headers */
#include "IMU.h"
#include "Control.h"

translation_outer_pid::translation_outer_pid()
:scaled_travel(15)
{
	x.name() = "X";
	y.name() = "Y";
}

translation_outer_pid::translation_outer_pid(const translation_outer_pid& other)
{
	{
		boost::mutex::scoped_lock lock(other.x_lock);
		x = other.x;
	}
	{
		boost::mutex::scoped_lock lock(other.y_lock);
		y = other.y;
	}
	{
		boost::mutex::scoped_lock lock(other.scaled_travel_lock);
		scaled_travel = other.scaled_travel;
	}
}



void translation_outer_pid::operator()(const blas::vector<double>& reference) throw(bad_control)
{
	// get attitude measurement
	IMU* imu = IMU::getInstance();
	blas::vector<double> euler(imu->get_euler());

	// get ned position/velocity
	blas::vector<double> position(imu->get_ned_position());
	blas::matrix<double> body_rotation(IMU::euler_to_rotation(euler));
	blas::vector<double> body_position_error(blas::prod(body_rotation, position - reference));
	blas::vector<double> body_velocity_error(blas::prod(body_rotation, imu->get_ned_velocity()));

	std::vector<double> log(body_position_error.begin(), body_position_error.end());
	log.insert(log.end(), body_velocity_error.begin(), body_velocity_error.end());
	LogFile::getInstance()->logData("Translation PID error", log);

	// roll pitch reference
	blas::vector<double> attitude_reference(2);
	attitude_reference.clear();
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.error().proportional() = body_position_error[0];
		x.error().derivative() = body_velocity_error[0];
		++(x.error());
		attitude_reference[1] = x.compute_pid();
	}
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.error().proportional() = body_position_error[1];
		y.error().derivative() = body_velocity_error[1];
		++(y.error());
		attitude_reference[0] = -y.compute_pid();
	}

	// get a normalized PID control signal
	Control::saturate(attitude_reference);

	// set the reference to a roll pitch orientation in radians
	set_control_effort(attitude_reference * scaled_travel_radians());
	// log the control effort
	LogFile::getInstance()->logData("Translation PID reference attitude", attitude_reference * scaled_travel_radians());
}

void translation_outer_pid::reset()
{
	for (boost::array<GPS_Filter, 3>::iterator it = pos_filters.begin(); it != pos_filters.end(); ++it)
		(*it).reset();
	for (boost::array<GPS_Filter, 3>::iterator it = vel_filters.begin(); it != vel_filters.end(); ++it)
		(*it).reset();

	x.reset();
	y.reset();
}

bool translation_outer_pid::runnable() const
{
	return true;
}

const std::string translation_outer_pid::PARAM_X_KP = "PID_X_KP";
const std::string translation_outer_pid::PARAM_X_KD = "PID_X_KD";
const std::string translation_outer_pid::PARAM_X_KI = "PID_X_KI";

const std::string translation_outer_pid::PARAM_Y_KP = "PID_Y_KP";
const std::string translation_outer_pid::PARAM_Y_KD = "PID_Y_KD";
const std::string translation_outer_pid::PARAM_Y_KI = "PID_Y_KI";

const std::string translation_outer_pid::PARAM_TRAVEL = "PID_TRAVEL";

std::vector<Parameter> translation_outer_pid::getParameters()
{
	std::vector<Parameter> plist;

	{
		boost::mutex::scoped_lock lock(x_lock);
		plist.push_back(Parameter(PARAM_X_KP, x.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KD, x.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KI, x.gains().integral(), heli::CONTROLLER_ID));
	}

	{
		boost::mutex::scoped_lock lock(y_lock);
		plist.push_back(Parameter(PARAM_Y_KP, y.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KD, y.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KI, y.gains().integral(), heli::CONTROLLER_ID));
	}
	scaled_travel_lock.lock();
	plist.push_back(Parameter(PARAM_TRAVEL, scaled_travel, heli::CONTROLLER_ID));
	scaled_travel_lock.unlock();
	return plist;
}

void translation_outer_pid::set_x_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().proportional() = kp;
	}
	message() << "Set x proportional gain to: " << kp;
}

void translation_outer_pid::set_x_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().derivative() = kd;
	}
	message() << "Set x derivative gain to: " << kd;
}

void translation_outer_pid::set_x_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().integral() = ki;
	}
	message() << "Set x integral gain to: " << ki;
}

void translation_outer_pid::set_y_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().proportional() = kp;
	}
	message() << "Set y proportional gain to: " << kp;
}

void translation_outer_pid::set_y_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().derivative() = kd;
	}
	message() << "Set y derivative gain to: " << kd;
}

void translation_outer_pid::set_y_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().integral() = ki;
	}
	message() << "Set y integral gain to: " << ki;
}

void translation_outer_pid::set_scaled_travel(double travel)
{
	{
		boost::mutex::scoped_lock lock(scaled_travel_lock);
		scaled_travel = travel;
	}
	message() << "Set travel to: " << travel;
}

rapidxml::xml_node<>* translation_outer_pid::get_xml_node(rapidxml::xml_document<>& doc)
{
	rapidxml::xml_node<> *pid_node = doc.allocate_node(rapidxml::node_element, "translation_outer_pid");
	{
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		rapidxml::xml_node<> *channel_node = doc.allocate_node(rapidxml::node_element, "channel");
		attr = doc.allocate_attribute("name", "x");
		channel_node->append_attribute(attr);
		pid_node->append_node(channel_node);

		{
			boost::mutex::scoped_lock lock(x_lock);
			node_value = doc.allocate_string(boost::lexical_cast<std::string>(x.gains().proportional()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "proportional");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(x.gains().derivative()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "derivative");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(x.gains().integral()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "integral");
			node->append_attribute(attr);
		}
		channel_node->append_node(node);

		channel_node = doc.allocate_node(rapidxml::node_element, "channel");
		attr = doc.allocate_attribute("name", "y");
		channel_node->append_attribute(attr);
		pid_node->append_node(channel_node);

		{
			boost::mutex::scoped_lock lock(y_lock);
			node_value = doc.allocate_string(boost::lexical_cast<std::string>(y.gains().proportional()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "proportional");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(y.gains().derivative()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "derivative");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(y.gains().integral()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "integral");
			node->append_attribute(attr);
		}
		channel_node->append_node(node);
		{
			// travel
			rapidxml::xml_node<> *node = NULL;
			char *node_value = NULL;
			node_value = doc.allocate_string(boost::lexical_cast<std::string>(scaled_travel_degrees()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "travel", node_value);
			pid_node->append_node(node);
		}
	}

	return pid_node;
}

void translation_outer_pid::parse_xml_node(rapidxml::xml_node<> *pid_params)
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

				if (channel_name == "X")
				{
					if (gain == "PROPORTIONAL")
						set_x_proportional(boost::lexical_cast<double>(gain_value));
					else if (gain == "DERIVATIVE")
						set_x_derivative(boost::lexical_cast<double>(gain_value));
					else if (gain == "INTEGRAL")
						set_x_integral(boost::lexical_cast<double>(gain_value));
					else
						warning() << __FILE__ << __LINE__ << "Unknown gain on x channel: " << gain;
				}
				else if (channel_name == "Y")
				{
					if (gain == "PROPORTIONAL")
						set_y_proportional(boost::lexical_cast<double>(gain_value));
					else if (gain == "DERIVATIVE")
						set_y_derivative(boost::lexical_cast<double>(gain_value));
					else if (gain == "INTEGRAL")
						set_y_integral(boost::lexical_cast<double>(gain_value));
					else
						warning() << __FILE__ << __LINE__ << "Unknown gain on pitch channel: " << gain;
				}
				else
					warning() << "Translation PID: Unknown channel: " << channel_name;
			}
		}
		else if(boost::to_upper_copy(std::string(channel->name())) == "TRAVEL")
		{
			std::string travel_value(channel->value());
			boost::trim(travel_value);
			set_scaled_travel_degrees(boost::lexical_cast<double>(travel_value));
		}
		else
			warning() << "Control::parse_pid(): unknown xml node " << channel->name();
	}
}


