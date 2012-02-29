/*******************************************************************************
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
 ******************************************************************************/

#ifndef RADIOCALIBRATION_H
#define RADIOCALIBRATION_H

/* Boost Headers */
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string.hpp>

/* STL Headers */
#include <iostream>	// for debugging.
#include <fstream>
#include <string>

/* RapidXML XML Parser */
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_print.hpp>

/* Project Headers */
#include "heli.h"
#include "Debug.h"

/**
 * @brief This class contains the calibration data for the pulse normalization functions
 *
 * The calibration data is stored in /etc/autopilot/Calibration.xml.  This file is
 * read when the class is constructed (when the program starts) and is written to
 * each time a new calibration message is sent from QGroundControl.
 *
 * XML parsing is done using <a href="http://rapidxml.sourceforge.net/">RapidXML</a>
 * which is a lightweight and fast header-only parser.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Aakash Vasudevan <avasudev@ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 * @author Hasitha Senanayake <senanaya@ualberta.ca>
 * @date July 14, 2011: Created class
 * @date October 21, 2011 : Added functionality for storing calibration data in XML file
 */
class RadioCalibration
{
public:
	/// Construct the class
	static RadioCalibration* getInstance();

	boost::array<uint16_t, 3> getAileron();
	boost::array<uint16_t, 3> getElevator();
	boost::array<uint16_t, 5> getThrottle();
	boost::array<uint16_t, 3> getRudder();
	boost::array<uint16_t, 2> getGyro();
	boost::array<uint16_t, 5> getPitch();
	/// Get position of flight mode switch
	boost::array<uint16_t, 3> getFlightMode();

	/**
	 * This function is called to change the calibration data.  After update the internal values,
	 * the configuration file is updated with the new values
	 * @param calibration_data vector of calibration info.  The contents must be aileron,
	 * elevator, throttle, rudder, gyro, pitch.
	 */
	void setCalibration(const std::vector<std::vector<uint16_t> >& calibration_data);

private:
	RadioCalibration();
	static RadioCalibration* _instance;

	/**
	 * Set the endpoints and center for the aileron servo
	 * @param setpoints new setpoint values
	 */
	void setAileron(const boost::array<uint16_t, 3>& setpoints);
	/**
	 * Set the endpoints and center for the aileron servo
	 * @param setpoints new setpoint values
	 */
	void setAileron(const std::vector<uint16_t>& setpoints);
	/**
	 * Set the endpoints and center for the elevator servo
	 * @param setpoints new setpoint values
	 */
	void setElevator(const boost::array<uint16_t, 3>& setpoints);
	/**
	 * Set the endpoints and center for the elevator servo
	 * @param setpoints new setpoint values
	 */
	void setElevator(const std::vector<uint16_t>& setpoints);
	/**
	 * Set the 5 point throttle curve
	 * @param setpoints new setpoint values
	 */
	void setThrottle(const boost::array<uint16_t, 5>& setpoints);
	/**
	 * Set the 5 point throttle curve
	 * @param setpoints new setpoint values
	 */
	void setThrottle(const std::vector<uint16_t>& setpoints);
	/**
	 * Set the rudder channel endpoints and center
	 * @param setpoints
	 */
	void setRudder(const boost::array<uint16_t, 3>& setpoints);
	/**
	 * Set the rudder channel endpoints and center
	 * @param setpoints new setpoint values
	 */
	void setRudder(const std::vector<uint16_t>& setpoints);
	/**
	 * Set the gyro mode switch endpoints
	 * @param setpoints new setpoint values
	 */
	void setGyro(const boost::array<uint16_t, 2>& setpoints);
	/**
	 * Set the gyro mode switch endpoints
	 * @param setpoints new setpoint values
	 */
	void setGyro(const std::vector<uint16_t>& setpoints);
	/**
	 * Set the 5 point pitch curve setpoints
	 * @param setpoints new setpoint values
	 */
	void setPitch(const boost::array<uint16_t, 5>& setpoints);
	/**
	 * Set the 5 point pitch curve setpoints
	 * @param setpoints new setpoint values
	 */
	void setPitch(const std::vector<uint16_t>& setpoints);

	boost::recursive_mutex calibration_lock;
	boost::mutex calibration_file_lock;
	boost::array<uint16_t, 2> gyro;

	boost::array<uint16_t, 3> aileron;
	boost::array<uint16_t, 3> elevator;
	boost::array<uint16_t, 3> rudder;

	boost::array<uint16_t, 5> throttle;
	boost::array<uint16_t, 5> pitch;

	boost::array<uint16_t, 3> flightMode;

	/// Read XML configuration file from heli::calibration_filename
	void loadFile();
	/**
	 * Determine type of xml node and apply approprate configuration to this class.
	 * @param setpoint pointer to xml node with configuration information.
	 */
	void parseSetpoint(const rapidxml::xml_node<> *setpoint);

	/**
	 * Write the calibration info into an xml file.  The file is stored in
	 * heli::calibration_filename.
	 */
	void saveFile();

	/**
	 * Converts a boost::array or std::vector of setpoints to a comma separated list
	 * @param setpoints array or vector of setpoints.  In fact any type can be used which
	 * has size() and operator[](int) as members and whose contents have operator<<
	 * defined.
	 */
	template <typename T>
	std::string toString(const T& setpoints);
};

template <typename T>
std::string RadioCalibration::toString(const T& setpoints)
{
	std::stringstream ss;
	for (int i=0, size=0; i<(size=setpoints.size()); i++)
	{
		ss << setpoints[i];
		if (i < size-1)
			ss << ", ";
	}

	return ss.str();
}
#endif //RADIOCALIBRATION_H
