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

#include "line.h"

/* STL Headers */
#include <math.h>

/* Project Headers */
#include "IMU.h"

line::line()
: start_location(blas::zero_vector<double>(3)),
  end_location(blas::zero_vector<double>(3)),
  hover_time(0)
{


}

const std::string line::PARAM_HOVER_TIME = "LIN_HOVER_TIME";
const std::string line::PARAM_X_TRAVEL = "LIN_X_TRAVEL";
const std::string line::PARAM_Y_TRAVEL = "LIN_Y_TRAVEL";
const std::string line::PARAM_SPEED = "LIN_SPEED";

void line::reset()
{
	set_start_location(IMU::getInstance()->get_ned_position());
	set_start_time();
	blas::vector<double> body_travel(3);
	body_travel(0) = get_x_travel();
	body_travel(1) = get_y_travel();
	set_end_location(get_start_location() + prod(IMU::getInstance()->get_heading_rotation(), body_travel));
}

std::vector<Parameter> line::getParameters() const
{
	std::vector<Parameter> plist;
	plist.push_back(Parameter(PARAM_HOVER_TIME, get_hover_time(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_X_TRAVEL, get_x_travel(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_Y_TRAVEL, get_y_travel(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_SPEED, get_speed(), heli::CONTROLLER_ID));
	return plist;
}

blas::vector<double> line::get_reference_position() const
{
	double elapsed_time = (boost::posix_time::microsec_clock::local_time() - get_start_time()).total_milliseconds()/1000.0;
	double flight_time = (get_speed() > 0 ? get_distance()/get_speed() : 0);
	double hover_time = get_hover_time();
	if (flight_time == 0 || elapsed_time <= hover_time)
	{
		return get_start_location();
	}
	else if ((elapsed_time - hover_time) <= flight_time)
	{
		elapsed_time -= hover_time;
		blas::vector<double> ned_velocity((get_end_location() - get_start_location())/flight_time);
		return get_start_location() + ned_velocity*elapsed_time;
	}
	else
	{
		return get_end_location();
	}
}

double line::get_distance() const
{
	return norm_2(get_end_location() - get_start_location());
}

rapidxml::xml_node<>* line::get_xml_node(rapidxml::xml_document<>& doc)
{
	rapidxml::xml_node<> *line_node = doc.allocate_node(rapidxml::node_element, "line");
	{
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_x_travel()).c_str());
		rapidxml::xml_node<> *param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "x_travel");
		param_node->append_attribute(attr);
		line_node->append_node(param_node);

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_y_travel()).c_str());
		param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "y_travel");
		param_node->append_attribute(attr);
		line_node->append_node(param_node);

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_speed()).c_str());
		param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "speed");
		param_node->append_attribute(attr);
		line_node->append_node(param_node);

		node_value = doc.allocate_string(boost::lexical_cast<std::string>(get_hover_time()).c_str());
		param_node = doc.allocate_node(rapidxml::node_element, "param", node_value);
		attr = doc.allocate_attribute("name", "hover");
		param_node->append_attribute(attr);
		line_node->append_node(param_node);
	}

	return line_node;
}

void line::parse_xml_node(rapidxml::xml_node<> *line_params)
{
	for (rapidxml::xml_node<> *param = line_params->first_node(); param; param = param->next_sibling())
	{
		if (boost::to_upper_copy(std::string(param->name())) == "PARAM")
		{
			rapidxml::xml_attribute<> *attr;
			for (attr = param->first_attribute(); attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
			std::string param_name(attr->value());
			boost::to_upper(param_name);
			std::string param_value(param->value());
			boost::trim(param_value);

			if (param_name == "X_TRAVEL")
				set_x_travel(boost::lexical_cast<double>(param_value));
			else if (param_name == "Y_TRAVEL")
				set_y_travel(boost::lexical_cast<double>(param_value));
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
