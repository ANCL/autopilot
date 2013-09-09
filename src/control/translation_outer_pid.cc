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
#include "Helicopter.h"

translation_outer_pid::translation_outer_pid()
: x(100),
  y(100),
  z(100),
  control_effort(3,0),
  control_derivative(3,0),
  control_2derivative(3,0),
  scaled_travel(15)
{
	x.name() = "X";
	y.name() = "Y";
	z.name() = "Z";

	LogFile::getInstance()->logHeader(heli::LOG_TRANS_PID_ERROR_STATES, "X_Prop, X_Deriv, X_Int, Y_Prop, Y_Deriv, Y_Int, Z_Prop, Z_Deriv, Z_Int");
	LogFile::getInstance()->logData(heli::LOG_TRANS_PID_ERROR_STATES, std::vector<double>());
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



void translation_outer_pid::operator()(const blas::vector<double>& reference,
		const blas::vector<double>& reference_derivative,
		const blas::vector<double>& reference_2derivative) throw(bad_control)
{
	// get attitude measurement
	IMU* imu = IMU::getInstance();
	blas::vector<double> euler(imu->get_euler());

	// get ned position/velocity
	blas::vector<double> position(imu->get_ned_position());
	blas::matrix<double> body_rotation(trans(IMU::euler_to_rotation(euler)));
	blas::vector<double> body_position_error(blas::prod(body_rotation, position - reference));
	blas::vector<double> body_velocity_error(blas::prod(body_rotation, imu->get_ned_velocity() - reference_derivative));
	blas::vector<double> body_reference_2derivative(blas::prod(body_rotation, reference_2derivative));


	blas::vector<double> control_effort(blas::zero_vector<double>(3)); // general control effort u^t
	std::vector<double> error_states; // log
	double mass = Helicopter::getInstance()->get_mass();
	{
		boost::mutex::scoped_lock lock(x_lock);
		error_states.push_back(x.error().proportional() = body_position_error[0]);
		error_states.push_back(x.error().derivative() = body_velocity_error[0]);
		error_states.push_back(++(x.error()));
		control_effort[0] = mass*body_reference_2derivative[0] + x.compute_pid();
	}
	{
		boost::mutex::scoped_lock lock(y_lock);
		error_states.push_back(y.error().proportional() = body_position_error[1]);
		error_states.push_back(y.error().derivative() = body_velocity_error[1]);
		error_states.push_back(++(y.error()));
		control_effort[1] = mass*body_reference_2derivative[1] + y.compute_pid();
	}
	{
		boost::mutex::scoped_lock lock(z_lock);
		error_states.push_back(z.error().proportional() = body_position_error[2]);
		error_states.push_back(z.error().derivative() = body_velocity_error[2]);
		error_states.push_back(++(z.error()));
		control_effort[2] = mass*body_reference_2derivative[2] + z.compute_pid();
	}


	LogFile::getInstance()->logData(heli::LOG_TRANS_PID_ERROR_STATES, error_states);

	// compute thrust and roll-pitch reference
	double gravity = Helicopter::getInstance()->get_gravity();
	blas::matrix<double> scaling_matrix(blas::zero_matrix<double>(3,3));
	scaling_matrix(0,1) = 1.0/mass/gravity;
	scaling_matrix(1,0) = -1.0/mass/gravity;
	scaling_matrix(2,2) = 1;
	blas::vector<double> physical_controls = blas::prod(scaling_matrix, control_effort);
	physical_controls(2) += mass*gravity;

	blas::vector<double> roll_pitch_reference(2,0);
	std::copy(roll_pitch_reference.begin(), physical_controls.begin(), physical_controls.end()-1);


//	Control::saturate(attitude_reference, scaled_travel_radians());

	// set the reference to a roll pitch orientation in radians
	set_control_effort(physical_controls);

	blas::matrix<double> Kp(blas::zero_matrix<double>(2,2));
	Kp(0,0) = x.gains().proportional();
	Kp(1,1) = y.gains().proportional();
	blas::matrix<double> Kd(blas::zero_matrix<double>(2,2));
	Kd(0,0) = x.gains().derivative();
	Kd(1,1) = y.gains().derivative();
	blas::matrix<double> Ki(blas::zero_matrix<double>(2,2));
	Ki(0,0) = x.gains().integral();
	Ki(1,1) = y.gains().integral();

	blas::vector<double> reference_3derivative(2,0);
	blas::vector<double> reference_4derivative(2,0);

	blas::vector<double> x_y_error(2,0);
	std::copy(x_y_error.begin(), body_position_error.begin(), body_position_error.end()-1);
	blas::vector<double> x_y_velocity_error(2,0);
	std::copy(x_y_velocity_error.begin(), body_velocity_error.begin(), body_velocity_error.end()-1);
	blas::vector<double> x_y_error_integral(2,0);
	x_y_error_integral(0) = x.error().integral();

	blas::vector<double> roll_pitch_reference_derivative(reference_3derivative*mass
			+ blas::prod((blas::prod(Kd,Kd)-Kp),x_y_velocity_error)
			+ blas::prod((blas::prod(Kd,Kp)-Ki),x_y_error)
			+ blas::prod(blas::prod(Kd,Ki),x_y_error_integral));

	blas::vector<double> roll_pitch_reference_2derivative(reference_4derivative*mass
			+ blas::prod(-blas::prod(blas::matrix<double>(blas::prod(Kd,Kd)),Kd)+blas::prod(Kd,Kp)*2-Ki, x_y_velocity_error)
			+ blas::prod(-blas::prod(Kd,Kp)+blas::prod(Kd,Ki)+blas::prod(Kp,Kp), x_y_error)
			+ blas::prod(-blas::prod(blas::matrix<double>(blas::prod(Kd,Kd)),Ki)+blas::prod(Kp,Ki),x_y_error_integral));
}

void translation_outer_pid::reset()
{
//	for (boost::array<GPS_Filter, 3>::iterator it = pos_filters.begin(); it != pos_filters.end(); ++it)
//		(*it).reset();
//	for (boost::array<GPS_Filter, 3>::iterator it = vel_filters.begin(); it != vel_filters.end(); ++it)
//		(*it).reset();

	x.reset();
	y.reset();
	z.reset();
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
	message() << "Set PID x proportional gain to: " << kp;
}

void translation_outer_pid::set_x_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().derivative() = kd;
	}
	message() << "Set PID x derivative gain to: " << kd;
}

void translation_outer_pid::set_x_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().integral() = ki;
	}
	message() << "Set PID x integral gain to: " << ki;
}

void translation_outer_pid::set_y_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().proportional() = kp;
	}
	message() << "Set PID y proportional gain to: " << kp;
}

void translation_outer_pid::set_y_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().derivative() = kd;
	}
	message() << "Set PID y derivative gain to: " << kd;
}

void translation_outer_pid::set_y_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().integral() = ki;
	}
	message() << "Set PID y integral gain to: " << ki;
}

void translation_outer_pid::set_scaled_travel(double travel)
{
	{
		boost::mutex::scoped_lock lock(scaled_travel_lock);
		scaled_travel = travel;
	}
	message() << "Set attitude reference limit to: " << travel;
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


