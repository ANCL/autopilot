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

#include "tail_sbf.h"

/* Project Headers */
#include "IMU.h"
#include "tail_sbf.h"
#include "Helicopter.h"
#include "Control.h"

/* STL Headers */
#include <math.h>

tail_sbf::tail_sbf()
: ned_x(10),
  ned_y(10)
{


}

void tail_sbf::reset()
{
	ned_x.reset();
	ned_y.reset();
}

bool tail_sbf::runnable() const
{
	return true;
}

void tail_sbf::operator()(const blas::vector<double>& reference) throw(bad_control)
{

	IMU* imu = IMU::getInstance();
	blas::vector<double> ned_position_error(imu->get_ned_position() - reference);
	blas::vector<double> ned_velocity_error(imu->get_ned_velocity());

	blas::vector<double> ned_control(3);
	ned_control.clear();
	std::vector<double> error_states;
	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		error_states.push_back(ned_x.error().proportional() = ned_position_error(0));
		error_states.push_back(ned_x.error().derivative() = ned_velocity_error(0));
		error_states.push_back(++(ned_x.error()));
		ned_control(0) = ned_x.compute_pid();
	}
	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		error_states.push_back(ned_y.error().proportional() = ned_position_error(1));
		error_states.push_back(ned_y.error().derivative() = ned_velocity_error(1));
		error_states.push_back(++(ned_y.error()));
		ned_control(1) = ned_y.compute_pid();
	}

	LogFile::getInstance()->logData(heli::LOG_TRANS_SBF_ERROR_STATES, error_states);

	double heading = imu->get_euler()(2);
	blas::matrix<double> Rz(3,3);
	Rz.clear();
	Rz(0,0) = cos(heading);
	Rz(0,1) = -sin(heading);
	Rz(1,0) = sin(heading);
	Rz(1,1) = cos(heading);
	Rz(2,2) = 1;

	Helicopter* bergen = Helicopter::getInstance();
	double m = bergen->get_mass();
	double g = bergen->get_gravity();
	ned_control(2) = -g;
	blas::vector<double> body_control(m*prod(trans(Rz),ned_control));
	double alpham = 0.04; // countertorque approximate slope
	double xt = abs(bergen->get_tail_hub_offset()(0));

	double theta_ref = atan(body_control(0)/body_control(2));
	double phi_ref = -atan((alpham*sqrt(pow(body_control(0),2)+pow(body_control(2),2))+xt*body_control(1))/(alpham*body_control(1) - xt*sqrt(pow(body_control(0),2)+pow(body_control(2),2))));

	blas::vector<double> attitude_reference(2);
	attitude_reference(0) = phi_ref;
	attitude_reference(1) = theta_ref;

	Control::saturate(attitude_reference, scaled_travel_radians());

	set_control_effort(attitude_reference);
}

const std::string tail_sbf::PARAM_X_KP = "SBF_X_KP";
const std::string tail_sbf::PARAM_X_KD = "SBF_X_KD";
const std::string tail_sbf::PARAM_X_KI = "SBF_X_KI";

const std::string tail_sbf::PARAM_Y_KP = "SBF_Y_KP";
const std::string tail_sbf::PARAM_Y_KD = "SBF_Y_KD";
const std::string tail_sbf::PARAM_Y_KI = "SBF_Y_KI";

const std::string tail_sbf::PARAM_TRAVEL = "SBF_TRAVEL";

std::vector<Parameter> tail_sbf::getParameters() const
{
	std::vector<Parameter> plist;

	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		plist.push_back(Parameter(PARAM_X_KP, ned_x.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KD, ned_x.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KI, ned_x.gains().integral(), heli::CONTROLLER_ID));
	}

	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		plist.push_back(Parameter(PARAM_Y_KP, ned_y.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KD, ned_y.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KI, ned_y.gains().integral(), heli::CONTROLLER_ID));
	}

	plist.push_back(Parameter(PARAM_TRAVEL, scaled_travel_degrees(), heli::CONTROLLER_ID));

	return plist;
}

void tail_sbf::set_x_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		ned_x.gains().proportional() = kp;
	}
	message() << "Set SBF x proportional gain to: " << kp;
}

void tail_sbf::set_x_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		ned_x.gains().derivative() = kd;
	}
	message() << "Set SBF x derivative gain to: " << kd;
}

void tail_sbf::set_x_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		ned_x.gains().integral() = ki;
	}
	message() << "Set SBF x integral gain to: " << ki;
}

void tail_sbf::set_y_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		ned_y.gains().proportional() = kp;
	}
	message() << "Set SBF y proportional gain to: " << kp;
}

void tail_sbf::set_y_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		ned_y.gains().derivative() = kd;
	}
	message() << "Set SBF y derivative gain to: " << kd;
}

void tail_sbf::set_y_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		ned_y.gains().integral() = ki;
	}
	message() << "Set SBF y integral gain to: " << ki;
}

void tail_sbf::set_scaled_travel(double travel)
{
	{
		boost::mutex::scoped_lock lock(scaled_travel_lock);
		scaled_travel = travel;
	}
	message() << "Set travel to: " << travel;
}

rapidxml::xml_node<>* tail_sbf::get_xml_node(rapidxml::xml_document<>& doc)
{
	rapidxml::xml_node<> *sbf_node = doc.allocate_node(rapidxml::node_element, "translation_outer_sbf");
	{
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		rapidxml::xml_node<> *channel_node = doc.allocate_node(rapidxml::node_element, "channel");
		attr = doc.allocate_attribute("name", "ned_x");
		channel_node->append_attribute(attr);
		sbf_node->append_node(channel_node);

		{
			boost::mutex::scoped_lock lock(ned_x_lock);
			node_value = doc.allocate_string(boost::lexical_cast<std::string>(ned_x.gains().proportional()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "proportional");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(ned_x.gains().derivative()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "derivative");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(ned_x.gains().integral()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "integral");
			node->append_attribute(attr);
		}
		channel_node->append_node(node);

		channel_node = doc.allocate_node(rapidxml::node_element, "channel");
		attr = doc.allocate_attribute("name", "ned_y");
		channel_node->append_attribute(attr);
		sbf_node->append_node(channel_node);

		{
			boost::mutex::scoped_lock lock(ned_y_lock);
			node_value = doc.allocate_string(boost::lexical_cast<std::string>(ned_y.gains().proportional()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "proportional");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(ned_y.gains().derivative()).c_str());
			node = doc.allocate_node(rapidxml::node_element, "gain", node_value);
			attr = doc.allocate_attribute("type", "derivative");
			node->append_attribute(attr);
			channel_node->append_node(node);

			node_value = doc.allocate_string(boost::lexical_cast<std::string>(ned_y.gains().integral()).c_str());
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
			sbf_node->append_node(node);
		}
	}

	return sbf_node;
}

void tail_sbf::parse_xml_node(rapidxml::xml_node<> *sbf_params)
{
	for (rapidxml::xml_node<> *channel = sbf_params->first_node(); channel; channel = channel->next_sibling())
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

				if (channel_name == "NED_X")
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
				else if (channel_name == "NED_Y")
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
					warning() << "Translation SBF: Unknown channel: " << channel_name;
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
