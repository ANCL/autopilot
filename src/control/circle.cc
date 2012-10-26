/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 *
 * This file is part of ANCL Autopilot.
nnnn *
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

#include "circle.h"

/* Boost Headers */
#include <boost/math/constants/constants.hpp>

/* STL Headers */
#include <math.h>

/* Project Headers */
#include "IMU.h"

circle::circle()
: radius(0),
  start_location(blas::zero_vector<double>(3)),
  center_location(blas::zero_vector<double>(3)),
  initial_angle(0)
{


}

const std::string circle::PARAM_HOVER_TIME = "CIR_HOVER_TIME";
const std::string circle::PARAM_RADIUS = "CIR_RADIUS";
const std::string circle::PARAM_SPEED = "CIR_SPEED";

void circle::reset()
{
	set_start_location(IMU::getInstance()->get_ned_position());
	set_start_time();
	set_center_location();

	blas::vector<double> initial_vector(get_start_location() - get_center_location());
	set_initial_angle(atan2(initial_vector(1), initial_vector(0)));
}

std::vector<Parameter> circle::getParameters() const
{
	std::vector<Parameter> plist;
	plist.push_back(Parameter(PARAM_HOVER_TIME, get_hover_time(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_RADIUS, get_radius(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_SPEED, get_speed(), heli::CONTROLLER_ID));
	return plist;
}

blas::vector<double> circle::get_reference_position() const
{
	double elapsed_time = (boost::posix_time::microsec_clock::local_time() - get_start_time()).total_milliseconds()/1000.0;
	double period = (get_speed() > 0 ? get_circumference()/get_speed() : 0);
	double hover_time = get_hover_time();
	if (period == 0 || elapsed_time <= hover_time)
	{
		return get_start_location();
	}
	else
	{
		elapsed_time -= hover_time;
		blas::vector<double> reference_position(get_center_location());
		static double pi = boost::math::constants::pi<double>();
		reference_position(0) += get_radius()*cos(2*pi*elapsed_time/period + get_initial_angle());
		reference_position(1) += get_radius()*sin(2*pi*elapsed_time/period + get_initial_angle());
		return reference_position;
	}
}

double circle::get_circumference() const
{
	return 2*boost::math::constants::pi<double>()*get_radius();
}

void circle::set_center_location()
{
	blas::vector<double> center(blas::zero_vector<double>(3));
	center(0) = get_radius();  // vector in body frame with origin at heli
	{
		boost::mutex::scoped_lock lock(center_location_lock);
		center_location = prod(IMU::getInstance()->get_heading_rotation(), center) + get_start_location();
	}
	message() << "Circle: center_location set to: " << center_location;
}

rapidxml::xml_node<>* circle::get_xml_node(rapidxml::xml_document<>& doc)
{
	rapidxml::xml_node<> *circle_node = doc.allocate_node(rapidxml::node_element, "circle");
	{
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_radius()).c_str());
		rapidxml::xml_node<> *param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "radius");
		param_node->append_attribute(attr);
		circle_node->append_node(param_node);

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_speed()).c_str());
		param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "speed");
		param_node->append_attribute(attr);
		circle_node->append_node(param_node);

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_hover_time()).c_str());
		param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "hover");
		param_node->append_attribute(attr);
		circle_node->append_node(param_node);
	}

	return circle_node;
}

void circle::parse_xml_node(rapidxml::xml_node<> *circle_params)
{
	for (rapidxml::xml_node<> *param = circle_params->first_node(); param; param = param->next_sibling())
	{
		if (boost::to_upper_copy(std::string(param->name())) == "PARAM")
		{
			rapidxml::xml_attribute<> *attr;
			for (attr = param->first_attribute(); attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
			std::string param_name(attr->value());
			boost::to_upper(param_name);
			std::string param_value(param->value());
			boost::trim(param_value);

			if (param_name == "RADIUS")
				set_radius(boost::lexical_cast<double>(param_value));
			else if (param_name == "HOVER")
				set_hover_time(boost::lexical_cast<double>(param_value));
			else if (param_name == "SPEED")
				set_speed(boost::lexical_cast<double>(param_value));
			else
				warning() << "Found unknown circle parameter";
		}
		else
			warning() << "Found unknown node in circle xml node";
	}
}
