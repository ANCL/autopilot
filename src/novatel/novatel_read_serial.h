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

/* STL Headers */
#include <vector>

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
	read_serial(const read_serial& other);
	void operator()();
	/**
	 * Initialize serial port COM2
	 */
	void initPort();
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
		OEM4OK,
		OEM4ERROR_TIMEOUT_HEADER,
		OEM4ERROR_TIMEOUT_DATA,
		OEM4ERROR_TIMEOUT_CRC,
		OEM4ERROR_CRC_MISMATCH,
		OEM4ERROR_MISSING_INPUT_ARGS,
		OEM4ERROR_MISSING_OUTPUT_SPACE,
		OEM4ERROR_NOT_RESPONSE,
		OEM4ERROR_COMMAND_FAILED,
		OEM4ERROR_CATASTROPHIC
	};

	///Responses to error codes
	enum ERROR_RESPONSES
	{
		STASHDATA=512,
		BAILOUT,
		DONOTHING
	};
private:
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

	void parse_header(const std::vector<uint8_t>& header);
	void parse_log(const std::vector<uint8_t>& log);

	///Used by compute_checksum function
	static unsigned long CRC32Value(int i);

	static const unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;
	const int MAXSERIALRECEIVESIZE;

	/// serial port file descriptor
	int fd_ser;

	uint parse_enum(const std::vector<uint8_t>& log, int offset = 0);
	template<typename FloatingType>
	blas::vector<FloatingType> parse_3floats(const std::vector<uint8_t>& log, int offset = 0);

};
template<typename FloatingType>
blas::vector<FloatingType> GPS::read_serial::parse_3floats(const std::vector<uint8_t>& log, int offset)
{
	raw_to_float<double>(log.begin() + offset, log.begin() + offset + 8);
}

#endif
