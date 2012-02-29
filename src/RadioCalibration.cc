/**************************************************************************
 * Copyright 2012 Bryan Godbolt, Hasitha Senanayake, Aakash Vasudevan
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

#include "RadioCalibration.h"

RadioCalibration::RadioCalibration()
{
  gyro[0] = 1050;
  gyro[1] = 1800;

  aileron[0] = 1000;
  aileron[1] = 1500;
  aileron[2] = 2000;

  elevator[0] = 1000;
  elevator[1] = 1500;
  elevator[2] = 2000;

  rudder[0] = 1000;
  rudder[1] = 1500;
  rudder[2] = 2000;

  throttle[0] = 1000;
  throttle[1] = 1250;
  throttle[2] = 1500;
  throttle[3] = 1750;
  throttle[4] = 2000;

  pitch[0] = 1000;
  pitch[1] = 1250;
  pitch[2] = 1500;
  pitch[3] = 1750;
  pitch[4] = 2000;

  flightMode[0] = 1000;
  flightMode[1] = 1500;
  flightMode[2] = 1620;

  loadFile();
}

RadioCalibration* RadioCalibration::_instance = NULL;

RadioCalibration* RadioCalibration::getInstance()
{
  if(!_instance)
      _instance = new RadioCalibration;

  return _instance;
}

void RadioCalibration::setCalibration(const std::vector<std::vector<uint16_t> >& calibration_data)
{
	setAileron(calibration_data[0]);
	setElevator(calibration_data[1]);
	setThrottle(calibration_data[2]);
	setRudder(calibration_data[3]);
	setGyro(calibration_data[4]);
	setPitch(calibration_data[5]);

	saveFile();
}

boost::array<uint16_t, 3> RadioCalibration::getAileron()
{
	calibration_lock.lock();
	boost::array<uint16_t, 3> ail(aileron);
	calibration_lock.unlock();
	return ail;
}

boost::array<uint16_t, 3> RadioCalibration::getElevator()
{
	calibration_lock.lock();
	boost::array<uint16_t, 3> ele(elevator);
	calibration_lock.unlock();
	return ele;
}

boost::array<uint16_t, 3> RadioCalibration::getRudder()
{
	calibration_lock.lock();
	boost::array<uint16_t, 3> rud(rudder);
	calibration_lock.unlock();
	return rud;
}


boost::array<uint16_t, 5> RadioCalibration::getThrottle()
{
	calibration_lock.lock();
	boost::array<uint16_t, 5> thr(throttle);
	calibration_lock.unlock();
	return thr;
}

boost::array<uint16_t, 5> RadioCalibration::getPitch()
{
	calibration_lock.lock();
	boost::array<uint16_t, 5> pit(pitch);
	calibration_lock.unlock();
	return pit;
}

boost::array<uint16_t, 2> RadioCalibration::getGyro()
{
	calibration_lock.lock();
	boost::array<uint16_t, 2> gyr(gyro);
	calibration_lock.unlock();
	return gyr;
}

boost::array<uint16_t, 3> RadioCalibration::getFlightMode()
{
        calibration_lock.lock();
        boost::array<uint16_t, 3> flight_Mode(flightMode);
        calibration_lock.unlock();
        return flight_Mode;
}

void RadioCalibration::setAileron(const boost::array<uint16_t, 3>& setpoints)
{
	calibration_lock.lock();
	aileron = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setAileron(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<3; i++)
		aileron[i] = setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setElevator(const boost::array<uint16_t, 3>& setpoints)
{
	calibration_lock.lock();
	elevator = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setElevator(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<3; i++)
		elevator[i] = setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setThrottle(const boost::array<uint16_t, 5>& setpoints)
{
	calibration_lock.lock();
	throttle = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setThrottle(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for(int i=0; i<5; i++)
	{
		throttle[i]=setpoints[i];
	}
	calibration_lock.unlock();
}

void RadioCalibration::setRudder(const boost::array<uint16_t, 3>& setpoints)
{
	calibration_lock.lock();
	rudder = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setRudder(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<3; i++)
		rudder[i] = setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setGyro(const boost::array<uint16_t, 2>& setpoints)
{
	calibration_lock.lock();
	gyro = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setGyro(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<2; i++)
		gyro[i]=setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setPitch(const boost::array<uint16_t, 5>& setpoints)
{
	calibration_lock.lock();
	pitch = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setPitch(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for(int i=0; i<5; i++)
	{
		pitch[i]=setpoints[i];
	}
	calibration_lock.unlock();
}

void RadioCalibration::loadFile()
{
//	std::string filename = "/etc/autopilot/Calibration.xml";
	if (boost::filesystem::exists(heli::calibration_filename))
	{
		/* Read contents of calibration file into char* */
		std::ifstream calibration_file;
		int length;
		calibration_file_lock.lock();
		calibration_file.open(heli::calibration_filename.c_str());      // open input file
		calibration_file.seekg(0, std::ios::end);    // go to the end
		length = calibration_file.tellg();           // report location (this is the lenght)
		calibration_file.seekg(0, std::ios::beg);    // go back to the beginning
		char *buffer = new char[length+1];    // allocate memory for a buffer of appropriate dimension
		calibration_file.read(buffer, length);       // read the whole file into the buffer
		calibration_file.close();
		calibration_file_lock.unlock();
		buffer[length] = 0;

		rapidxml::xml_document<> calibration_xml;
		calibration_xml.parse<0>(buffer);

		rapidxml::xml_node<> *root_node = calibration_xml.first_node();

		if (std::string(root_node->name()) != "channels")
		{
			critical() << "RadioCalibration::loadFile: Unexpected file format.  Cannot load radio calibration.";
			return;
		}

		rapidxml::xml_node<> *setpoint;
		while ((setpoint = root_node->first_node()))
		{
			parseSetpoint(setpoint);
			root_node->remove_first_node();
		}
	}
}

void RadioCalibration::parseSetpoint(const rapidxml::xml_node<> *setpoint)
{
    std::vector<uint16_t> setpoints;
    std::string value(setpoint->value());
    unsigned int pos = 0;

    while ((pos = value.find(',')) != std::string::npos)
    {
    	std::string s(value.substr(0,pos));
    	boost::algorithm::trim(s);
    	setpoints.push_back(boost::lexical_cast<uint16_t>(s));
    	value = value.substr(pos+1);
    }
    boost::algorithm::trim(value);
    setpoints.push_back(boost::lexical_cast<uint16_t>(value));

    if (std::string(setpoint->name()) == "threeSetpoint")
    {
        if (setpoints.empty())
        	for (int i=0; i<3; i++)
        		setpoints.push_back(0);

        /* Find name attribute */

        rapidxml::xml_attribute<> *attr;
        std::string name;
        for (attr = setpoint->first_attribute();
        		attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
        name = std::string(attr->value());

        /* Set appropriate channel */
        boost::to_upper(name);
        if (name == "AILERON")
        	setAileron(setpoints);
        else if (name == "ELEVATOR")
        	setElevator(setpoints);
        else if (name == "RUDDER")
        	setRudder(setpoints);
    }
    else if(std::string(setpoint->name())== "twoSetpoint")
    {
    	if(setpoints.empty())
    		for(int j=0;j<2;j++)
    			setpoints.push_back(0);

    	rapidxml::xml_attribute<> *attr;
    	std::string name;
    	for (attr = setpoint->first_attribute();
    	        		attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
    	name = std::string(attr->value());
    	boost::to_upper(name);
    	if(name=="GYRO")
    		setGyro(setpoints);
    }
    else if(std::string(setpoint->name())== "fiveSetpoint")
	{
		if(setpoints.empty())
			for(int j=0;j<5;j++)
				setpoints.push_back(0);

		rapidxml::xml_attribute<> *attr;
		std::string name;
		for (attr = setpoint->first_attribute();
						attr && std::string(attr->name()) != "name"; attr = attr->next_attribute());
		name = std::string(attr->value());
		boost::to_upper(name);
		if(name=="THROTTLE")
			setThrottle(setpoints);
		else if(name=="PITCH")
			setPitch(setpoints);
	}
}

void RadioCalibration::saveFile()
{

	/* Create document and root node */
	rapidxml::xml_document<> calibration_xml;
	rapidxml::xml_node<> *root_node = calibration_xml.allocate_node(rapidxml::node_element, "channels");
	calibration_xml.append_node(root_node);

	/* Add Nodes for Channels */
	rapidxml::xml_node<> *node;
	char *node_value;
	rapidxml::xml_attribute<> *attr;

	/* Aileron*/
	node_value = calibration_xml.allocate_string(toString(getAileron()).c_str());
	node = calibration_xml.allocate_node(rapidxml::node_element, "threeSetpoint", node_value);
	attr = calibration_xml.allocate_attribute("name", "Aileron");
	node->append_attribute(attr);
	attr = calibration_xml.allocate_attribute("number", "1");
	node->append_attribute(attr);
	root_node->append_node(node);

	/* Elevator */
	node_value = calibration_xml.allocate_string(toString(getElevator()).c_str());
	node = calibration_xml.allocate_node(rapidxml::node_element, "threeSetpoint", node_value);
	attr = calibration_xml.allocate_attribute("name", "Elevator");
	node->append_attribute(attr);
	attr = calibration_xml.allocate_attribute("number", "2");
	node->append_attribute(attr);
	root_node->append_node(node);

	/* Throttle */
	node_value = calibration_xml.allocate_string(toString(getThrottle()).c_str());
	node = calibration_xml.allocate_node(rapidxml::node_element, "fiveSetpoint", node_value);
	attr = calibration_xml.allocate_attribute("name", "Throttle");
	node->append_attribute(attr);
	attr = calibration_xml.allocate_attribute("number", "3");
	node->append_attribute(attr);
	root_node->append_node(node);

	/* Rudder */
	node_value = calibration_xml.allocate_string(toString(getRudder()).c_str());
	node = calibration_xml.allocate_node(rapidxml::node_element, "threeSetpoint", node_value);
	attr = calibration_xml.allocate_attribute("name", "Rudder");
	node->append_attribute(attr);
	attr = calibration_xml.allocate_attribute("number", "4");
	node->append_attribute(attr);
	root_node->append_node(node);

	/* Gyro */
	node_value = calibration_xml.allocate_string(toString(getGyro()).c_str());
	node = calibration_xml.allocate_node(rapidxml::node_element, "twoSetpoint", node_value);
	attr = calibration_xml.allocate_attribute("name", "Gyro");
	node->append_attribute(attr);
	attr = calibration_xml.allocate_attribute("number", "5");
	node->append_attribute(attr);
	root_node->append_node(node);

	/* Pitch*/
	node_value = calibration_xml.allocate_string(toString(getPitch()).c_str());
	node = calibration_xml.allocate_node(rapidxml::node_element, "fiveSetpoint", node_value);
	attr = calibration_xml.allocate_attribute("name", "Pitch");
	node->append_attribute(attr);
	attr = calibration_xml.allocate_attribute("number", "6");
	node->append_attribute(attr);
	root_node->append_node(node);

//	std::cout << "RadioCalibration: calib_file=" << std::endl << calibration_xml << std::endl;

	std::ofstream calibration_file;
	calibration_file_lock.lock();
	calibration_file.open(heli::calibration_filename.c_str());
	calibration_file << calibration_xml;
	calibration_file.close();
	calibration_file_lock.unlock();
}

