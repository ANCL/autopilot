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

#ifndef NOVATEL_READ_SERIAL_H_
#define NOVATEL_READ_SERIAL_H_

/* c headers */
#include <stdint.h>
#include <math.h>

/* STL Headers */
#include <vector>

/* Boost Headers */
#include <boost/assign.hpp>
// this scope only pollutes the global namespace in a minimal way consistent with the stl global operators
using namespace boost::assign;
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/date_time/posix_time/posix_time.hpp>

/* Project Headers */
#include "GPS.h"

	/**
	 * @brief Class to send commands to, and receive data from the GPS unit.
	 * The class parses the incoming data and stores the message in a
	 * double vector.
	 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
	 * @author Aakash Vasudevan <avasudev@ualberta.ca>
	 * @date April 27, 2012: rewrote and refactored to integrate novatel with gx3
	 */
class GPS::read_serial
{
public:
	read_serial();

	void operator()();

	/**
	 * Send the log command for the required data
	 */
	void requestData();
	/**
	 * Function to read data from the serial port and send the output data to
	 * GPS class
	 * Also, tracks the number of successive GPS data with solution type less than 34 through
	 * the variables GPS::_pos_count and GPS::_vel_count
	 */
	void readPort();


	/// All possible errors that could be encountered when receiving data from GPS unit.
	enum OEM4_RETURN_CODES
	{
		OEM4OK = 1,
		OEM4_CRC_MISMATCH = 8
	};


private:
	/**
	 * Initialize serial port COM2
	 */
	void initPort();

	void reinitialize();

	/** Sends the log command to the GPS unit.  Automatically appends
	 *  the CRC by determining length of header and subsequent data
	 */
	void send_log_command();

	/** Send the 'unlog' command to the GPS unit. Automatically appends
	 * the CRC by determining the length of the header and subsequent data
	 * @param binhd const reference to the object of class oem4_binary_header (must be initialized appropriately)
	 * @param data const reference to the object of class unlog_data (must be initialized)
	 */
	void send_unlog_command();

	static std::vector<uint8_t> generate_header(uint16_t message_id, uint16_t message_length);

	static std::vector<uint8_t> compute_checksum(const std::vector<uint8_t>& message);

	/// parse the header and append the relevant field to the log
	void parse_header(const std::vector<uint8_t>& header, std::vector<double>& log);
	/// parse the message and append the relevant fields to the log
	void parse_log(const std::vector<uint8_t>& data, std::vector<double>& log);

	///Used by compute_checksum function
	static unsigned long CRC32Value(int i);

	static const unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;

	/// serial port file descriptor
	int fd_ser;

	/// extract an enum field from the novatel message
	uint parse_enum(const std::vector<uint8_t>& log, int offset = 0);
	/// extract a 3 vector of floating point type (double of float) from the novatel message
	template<typename FloatingType>
	static blas::vector<FloatingType> parse_3floats(const std::vector<uint8_t>& log, int offset = 0);

	/// rotate vectors in ecef into ned frame using the llh position parameter
	template<typename FloatingType>
	static blas::vector<FloatingType> ecef_to_ned(const blas::vector<FloatingType>& ecef, const blas::vector<double>& llh);

	/// convert ecef position measurement into llh
	static blas::vector<double> ecef_to_llh(const blas::vector<double>& ecef);

	/// check whether the header is from a response message
	static bool is_response(const std::vector<uint8_t>& header);

	/// stores the last time data was successfully received (for error handling)
	boost::posix_time::ptime last_data;

};
template<typename FloatingType>
blas::vector<FloatingType> GPS::read_serial::parse_3floats(const std::vector<uint8_t>& log, int offset)
{
	blas::vector<FloatingType> floats(3);
	for (int i=0; i<3; i++)
		floats[i] = raw_to_float<FloatingType>(log.begin() + offset + sizeof(FloatingType)*i,
				log.begin() + offset + sizeof(FloatingType)*(i+1));
	return floats;
}

template<typename FloatingType>
blas::vector<FloatingType> GPS::read_serial::ecef_to_ned(const blas::vector<FloatingType>& ecef, const blas::vector<double>& llh)
{
	blas::matrix<double> ned_rotation(3,3);
	ned_rotation(0,0) = -sin(llh[0])*cos(llh[1]);
	ned_rotation(0,1) = -sin(llh[0])*sin(llh[1]);
	ned_rotation(0,2) = cos(llh[0]);
	ned_rotation(1,0) = -sin(llh[1]);
	ned_rotation(1,1) = cos(llh[1]);
	ned_rotation(1,2) = 0;
	ned_rotation(2,0) = -cos(llh[0])*cos(llh[1]);
	ned_rotation(2,1) = -cos(llh[0])*sin(llh[1]);
	ned_rotation(2,2) = -sin(llh[0]);

	return prod(ned_rotation, ecef);
}

#endif
