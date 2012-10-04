/**************************************************************************
 * Copyright 2012 Bryan Godbolt, Hasitha Senanayake
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

#include "Helicopter.h"

/* RapidXML XML Parser */
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_print.hpp>

/* Boost Headers */
#include <boost/lexical_cast.hpp>

/* STL Headers */
#include <fstream>

Helicopter::Helicopter()
	:radio_cal_data(RadioCalibration::getInstance()),
	 out(servo_switch::getInstance()),
	 mass(13.65),
	 gravity(9.8),
	 main_hub_offset(3),
	 tail_hub_offset(3),
	 inertia(3,3)
{
	main_hub_offset.clear();
	main_hub_offset(2) = -0.32;

	tail_hub_offset.clear();
	tail_hub_offset(0) = -1.06;

	inertia(0,0) = 0.36;
	inertia(1,1) = 1.48;
	inertia(2,2) = 1.21;

	loadFile();
}

Helicopter* Helicopter::_instance = NULL;

Helicopter* Helicopter::getInstance()
{
  if(!_instance)
      _instance = new Helicopter;

  return _instance;
}

const std::string Helicopter::PARAM_MASS = "Mass";

const std::string Helicopter::PARAM_MAIN_OFFSET_X = "Main_X";
const std::string Helicopter::PARAM_MAIN_OFFSET_Y = "Main_Y";
const std::string Helicopter::PARAM_MAIN_OFFSET_Z = "Main_Z";

const std::string Helicopter::PARAM_TAIL_OFFSET_X = "Tail_X";
const std::string Helicopter::PARAM_TAIL_OFFSET_Y = "Tail_Y";
const std::string Helicopter::PARAM_TAIL_OFFSET_Z = "Tail_Z";

const std::string Helicopter::PARAM_INERTIA_X = "J_X";
const std::string Helicopter::PARAM_INERTIA_Y = "J_Y";
const std::string Helicopter::PARAM_INERTIA_Z = "J_Z";

std::vector<Parameter> Helicopter::getParameters()
{
	std::vector<Parameter> plist;

	plist.push_back(Parameter(PARAM_MASS, get_mass(), heli::HELICOPTER_ID));

	plist.push_back(Parameter(PARAM_MAIN_OFFSET_X, get_main_hub_offset()(0), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_MAIN_OFFSET_Y, get_main_hub_offset()(1), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_MAIN_OFFSET_Z, get_main_hub_offset()(2), heli::HELICOPTER_ID));

	plist.push_back(Parameter(PARAM_TAIL_OFFSET_X, get_tail_hub_offset()(0), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_TAIL_OFFSET_Y, get_tail_hub_offset()(1), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_TAIL_OFFSET_Z, get_tail_hub_offset()(2), heli::HELICOPTER_ID));

	plist.push_back(Parameter(PARAM_INERTIA_X, get_inertia()(0,0), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_INERTIA_Y, get_inertia()(1,1), heli::HELICOPTER_ID));
	plist.push_back(Parameter(PARAM_INERTIA_Z, get_inertia()(2,2), heli::HELICOPTER_ID));

	return plist;
}

void Helicopter::setParameter(Parameter p)
{
	std::string param_id(p.getParamID());
	boost::trim(param_id);

	if (param_id == PARAM_MASS)
		set_mass(p.getValue());

	else if (param_id == PARAM_MAIN_OFFSET_X)
		set_main_hub_offset_x(p.getValue());
	else if (param_id == PARAM_MAIN_OFFSET_Y)
		set_main_hub_offset_y(p.getValue());
	else if (param_id == PARAM_MAIN_OFFSET_Z)
		set_main_hub_offset_z(p.getValue());

	else if (param_id == PARAM_TAIL_OFFSET_X)
		set_tail_hub_offset_x(p.getValue());
	else if (param_id == PARAM_TAIL_OFFSET_Y)
		set_tail_hub_offset_y(p.getValue());
	else if (param_id == PARAM_TAIL_OFFSET_Z)
		set_tail_hub_offset_z(p.getValue());

	else if (param_id == PARAM_INERTIA_X)
		set_inertia_x(p.getValue());
	else if (param_id == PARAM_INERTIA_Y)
		set_inertia_y(p.getValue());
	else if (param_id == PARAM_INERTIA_Z)
		set_inertia_z(p.getValue());
	else
		debug() << "Helicopter: Received unknown parameter.";

	saveFile();
}

void Helicopter::saveFile() const
{
	rapidxml::xml_document<> config_file_xml;
	rapidxml::xml_node<> *root_node = config_file_xml.allocate_node(rapidxml::node_element, "physical_params");
	config_file_xml.append_node(root_node);

	{
		rapidxml::xml_node<> *node = NULL;
		char *node_value = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(get_mass()).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "scalar", node_value);
		attr = config_file_xml.allocate_attribute("name", "mass");
		node->append_attribute(attr);
		root_node->append_node(node);
	}

	{
		rapidxml::xml_node<> *vector_node = config_file_xml.allocate_node(rapidxml::node_element, "vector");

		char *node_value = NULL;
		rapidxml::xml_node<> *node = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		attr = config_file_xml.allocate_attribute("name", "main_hub_offset");
		vector_node->append_attribute(attr);

		blas::vector<double> main(get_main_hub_offset());
		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(main(0)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "x");
		node->append_attribute(attr);
		vector_node->append_node(node);

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(main(1)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "y");
		node->append_attribute(attr);
		vector_node->append_node(node);

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(main(2)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "z");
		node->append_attribute(attr);
		vector_node->append_node(node);

		root_node->append_node(vector_node);
	}

	{
		rapidxml::xml_node<> *vector_node = config_file_xml.allocate_node(rapidxml::node_element, "vector");

		char *node_value = NULL;
		rapidxml::xml_node<> *node = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		attr = config_file_xml.allocate_attribute("name", "tail_hub_offset");
		vector_node->append_attribute(attr);

		blas::vector<double> tail(get_tail_hub_offset());
		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(tail(0)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "x");
		node->append_attribute(attr);
		vector_node->append_node(node);

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(tail(1)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "y");
		node->append_attribute(attr);
		vector_node->append_node(node);

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(tail(2)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "z");
		node->append_attribute(attr);
		vector_node->append_node(node);

		root_node->append_node(vector_node);
	}

	{
		rapidxml::xml_node<> *vector_node = config_file_xml.allocate_node(rapidxml::node_element, "vector");

		char *node_value = NULL;
		rapidxml::xml_node<> *node = NULL;
		rapidxml::xml_attribute<> *attr = NULL;

		attr = config_file_xml.allocate_attribute("name", "inertia");
		vector_node->append_attribute(attr);

		blas::banded_matrix<double> inertia(get_inertia());
		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(inertia(0,0)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "x");
		node->append_attribute(attr);
		vector_node->append_node(node);

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(inertia(1,1)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "y");
		node->append_attribute(attr);
		vector_node->append_node(node);

		node_value = config_file_xml.allocate_string(boost::lexical_cast<std::string>(inertia(2,2)).c_str());
		node = config_file_xml.allocate_node(rapidxml::node_element, "coordinate", node_value);
		attr = config_file_xml.allocate_attribute("name", "z");
		node->append_attribute(attr);
		vector_node->append_node(node);

		root_node->append_node(vector_node);
	}

	std::ofstream config_file;
	config_file_lock.lock();
	config_file.open(heli::physical_param_filename.c_str());
	config_file << config_file_xml;
	config_file.close();
	config_file_lock.unlock();
}

uint16_t Helicopter::norm2pulse(double norm, boost::array<uint16_t, 3> setpoint)
{
        uint16_t pulse = 0;
        if(setpoint[2] > setpoint[0])
          {
            if(norm >= 0)
                pulse = setpoint[1] + (norm * (setpoint[2] - setpoint[1]));
            else
                pulse = setpoint[1] - (norm * (setpoint[0] - setpoint[1]));
          }
        else if(setpoint[2] < setpoint[0])
          {
            if(norm >= 0)
                pulse = setpoint[1] - (norm * (setpoint[1] - setpoint[2]));
            else
                pulse = setpoint[1] + (norm * (setpoint[1] - setpoint[0]));
          }
        return pulse;
}

uint16_t Helicopter::norm2pulse(double norm, boost::array<uint16_t, 2> setpoint)
{
  uint16_t pulse = 0;
    {
      if(norm == 0)
        pulse = setpoint[0];
      else
        pulse = setpoint[1];
    }
  return pulse;
}

uint16_t Helicopter::norm2pulse(double norm, boost::array<uint16_t, 5> setpoint)
{
  uint16_t pulse = 1.5;
  if(setpoint[4] > setpoint[0])
    {
      if(norm <= 0.25 && norm >= 0)
          pulse = (norm/0.25) * (setpoint[1] - setpoint[0]) + setpoint[0];
      else if(norm <= 0.50 && norm > 0.25)
          pulse = ((norm - 0.25)/0.25) * (setpoint[2] - setpoint[1]) + setpoint[1];
      else if(norm <= 0.75 && norm > 0.50)
          pulse = ((norm - 0.50)/0.25) * (setpoint[3] - setpoint[2]) + setpoint[2];
      else if(norm <= 1.0 && norm > 0.75)
          pulse = ((norm - 0.75)/0.25) * (setpoint[4] - setpoint[3]) + setpoint[3];
    }
  else if(setpoint[4] < setpoint[0])
    {
    if(norm <= 0.25 && norm >= 0)
        pulse = setpoint[0] - (norm/0.25) * (setpoint[0] - setpoint[1]);
    else if(norm <= 0.50 && norm > 0.25)
        pulse = setpoint[1] - ((norm - 0.25)/0.25) * (setpoint[1] - setpoint[2]);
    else if(norm <= 0.75 && norm > 0.50)
        pulse = setpoint[2] - ((norm - 0.50)/0.25) * (setpoint[2] - setpoint[3]);
    else if(norm <= 1.0 && norm > 0.75)
        pulse = setpoint[3] - ((norm - 0.75)/0.25) * (setpoint[3] - setpoint[4]);
    }
  return pulse;
}

boost::array<uint16_t, 6> Helicopter::setScaled(boost::array<double, 6> norm)
{
  boost::array<uint16_t, 6> pulse;

  pulse[AILERON] = setAileron(norm[0]);
  pulse[ELEVATOR] = setElevator(norm[1]);
  pulse[THROTTLE] = setThrottle(norm[2]);
  pulse[RUDDER] = setRudder(norm[3]);
  pulse[GYRO] = setGyro(norm[4]);
  pulse[PITCH] = setPitch(norm[5]);

  return pulse;
}

std::vector<uint16_t> Helicopter::setScaled(std::vector<double> norm)
{
  std::vector<uint16_t> pulse(6);

  pulse[AILERON] = setAileron(norm[0]);
  pulse[ELEVATOR] = setElevator(norm[1]);
  pulse[THROTTLE] = setThrottle(norm[2]);
  pulse[RUDDER] = setRudder(norm[3]);
  pulse[GYRO] = setGyro(norm[4]);
  pulse[PITCH] = setPitch(norm[5]);

  return pulse;
}

uint16_t Helicopter::setAileron(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getAileron());
	out->setRaw(heli::CH1, pulse);
	return pulse;
}

uint16_t Helicopter::setElevator(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getElevator());
	out->setRaw(heli::CH2, pulse);
	return pulse;
}

uint16_t Helicopter::setThrottle(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getThrottle());
	out->setRaw(heli::CH3, pulse);
	return pulse;
}

uint16_t Helicopter::setRudder(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getRudder());
	out->setRaw(heli::CH4, pulse);
	return pulse;
}

uint16_t Helicopter::setGyro(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getGyro());
	out->setRaw(heli::CH5, pulse);
	return pulse;
}

uint16_t Helicopter::setPitch(double norm)
{
	uint16_t pulse = norm2pulse(norm, radio_cal_data->getPitch());
	out->setRaw(heli::CH6, pulse);
	return pulse;
}

void Helicopter::loadFile()
{
	if (!boost::filesystem::exists(heli::physical_param_filename))
	{
		warning() << "Helicopter: Cannot find physical parameter xml file: " << heli::physical_param_filename;
		return;
	}

	/* Read contents of configuration file into char* */
	std::ifstream config_file;
	int length;
	config_file_lock.lock();
	config_file.open(heli::physical_param_filename.c_str());      // open input file
	config_file.seekg(0, std::ios::end);    // go to the end
	length = config_file.tellg();           // report location (this is the length)
	config_file.seekg(0, std::ios::beg);    // go back to the beginning
	char *config_file_buffer = new char[length+1];    // allocate memory for a buffer of appropriate dimension
	config_file.read(config_file_buffer, length);       // read the whole file into the buffer
	config_file.close();
	config_file_lock.unlock();
	config_file_buffer[length] = 0;

	rapidxml::xml_document<> config_file_xml;
	config_file_xml.parse<0>(config_file_buffer);

	rapidxml::xml_node<> *root_node = config_file_xml.first_node();

	if (boost::to_upper_copy(std::string(root_node->name())) != "PHYSICAL_PARAMS")
	{
		critical() << "Helicopter::loadFile() Unknown file format.  Cannot load physical parameters.";
		return;
	}

	for (rapidxml::xml_node<> *node = root_node->first_node(); node; node = node->next_sibling())
	{
		std::string node_name(node->name());
		if (boost::to_upper_copy(std::string(node_name)) == "SCALAR")
			parse_scalar_node(node);
		else if (boost::to_upper_copy(std::string(node_name)) == "VECTOR")
			parse_vector_node(node);
		else
			warning() << __FILE__ << __LINE__ << "Found unknown node: " << node_name;
	}
}

void Helicopter::parse_scalar_node(rapidxml::xml_node<> *scalar_node)
{

	std::string scalar_value(scalar_node->value());
	boost::trim(scalar_value);

	rapidxml::xml_attribute<> *attr;
	for (attr = scalar_node->first_attribute(); attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
	std::string name(attr->value());
	boost::to_upper(name);
	if (name == "MASS")
		set_mass(boost::lexical_cast<double>(scalar_value));
	else
		warning() << "Found Unknown Scalar value in physical parameter file";
}

void Helicopter::parse_vector_node(rapidxml::xml_node<> *vector_node)
{
	rapidxml::xml_attribute<> *attr;
	for (attr = vector_node->first_attribute(); attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
	std::string vector_name(attr->value());
	boost::to_upper(vector_name);

	for (rapidxml::xml_node<> *coordinate = vector_node->first_node(); coordinate; coordinate = coordinate->next_sibling())
		{
			if (boost::to_upper_copy(std::string(coordinate->name())) == "COORDINATE")
			{
				rapidxml::xml_attribute<> *attr;
				for (attr = coordinate->first_attribute(); attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
				std::string coordinate_name(attr->value());
				boost::to_upper(coordinate_name);
				std::string coordinate_value(coordinate->value());
				boost::to_upper(coordinate_value);

				if (vector_name == "MAIN_HUB_OFFSET")
				{
					if (coordinate_name == "X")
						set_main_hub_offset_x(boost::lexical_cast<double>(coordinate_value));
					else if (coordinate_name == "Y")
						set_main_hub_offset_y(boost::lexical_cast<double>(coordinate_value));
					else if (coordinate_name == "Z")
						set_main_hub_offset_z(boost::lexical_cast<double>(coordinate_value));
				}
				else if (vector_name == "TAIL_HUB_OFFSET")
				{
					if (coordinate_name == "X")
						set_tail_hub_offset_x(boost::lexical_cast<double>(coordinate_value));
					else if (coordinate_name == "Y")
						set_tail_hub_offset_y(boost::lexical_cast<double>(coordinate_value));
					else if (coordinate_name == "Z")
						set_tail_hub_offset_z(boost::lexical_cast<double>(coordinate_value));
				}
				else if (vector_name == "INERTIA")
				{
					if (coordinate_name == "X")
						set_inertia_x(boost::lexical_cast<double>(coordinate_value));
					else if (coordinate_name == "Y")
						set_inertia_y(boost::lexical_cast<double>(coordinate_value));
					else if (coordinate_name == "Z")
						set_inertia_z(boost::lexical_cast<double>(coordinate_value));
				}
			}
			else
				warning() << "parse_pid(): unknown xml node " << coordinate->name();
		}
}

double Helicopter::get_main_collective() const
{
	uint16_t pitch_servo = servo_switch::getInstance()->getRaw(heli::CH6);

	const uint16_t pitch_neutral = 1880;

	if (pitch_servo < pitch_neutral)
		return -0.017113*pitch_servo + 31.97;
	else
		return -0.0082034*pitch_servo + 15.349;
}
