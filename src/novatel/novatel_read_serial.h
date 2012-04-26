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

/* Project Headers */
#include "GPS.h"

	/**
	 * @brief Class to send commands to, and receive data from the GPS unit.
	 * The class parses the incoming data and stores the message in a
	 * double vector.
	 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
	 * @author Aakash Vasudevan <avasudev@ualberta.ca>
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

	class oem4_binary_header;
	/**
	 * @brief Class to send the 'log' command to the GPS unit
	 */
	class log_command_data
	{
	public:
		log_command_data();
		friend Debug& operator<<(Debug& dbg, const log_command_data& logcom);
	private:
		unsigned long	port;
	    unsigned short	msgID;
	    unsigned char	msgtype;
	    unsigned char	reserved;
	    unsigned long	trigger;
	    double			period;
	    double			offset;
	    unsigned long	hold;

	};

	/**
	 * @brief Class to send the 'unlog' command to GPS unit
	 */
	class unlog_command_data
	{
	public:
		unlog_command_data();

	private:
		unsigned long port;
		unsigned short msgId;
		unsigned char msgtype;
		unsigned char reserved;
	};

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
	/** Sends the 'log' command to the GPS unit.  Automatically appends
	 *  the CRC by determining length of header and subsequent data
	 *  @param binhdr const reference to the object of class oem4_binary_header (must be initialized appropriately)
	 *  @param data const reference to the object of class unlog_data (must be initialized)
	 */
	void sendLogCommand(const oem4_binary_header& binhdr, const log_command_data& data);

	/** Send the 'unlog' command to the GPS unit. Automatically appends
	 * the CRC by determining the length of the header and subsequent data
	 * @param binhd const reference to the object of class oem4_binary_header (must be initialized appropriately)
	 * @param data const reference to the object of class unlog_data (must be initialized)
	 */
	void sendUnlogCommand(const oem4_binary_header& binhd, const unlog_command_data& data);

	/**
	 *  Calculation of the CRC32 for a block separated into
	 *  binary header and data segments.
	 *  @param binhdr const reference to the object of class oem4_binary_header (must be initialized according to the command)
	 *  @param data pointer to the array containing the data of the log message
	 *  @return checksum of the message (CRC32)
	 */
	unsigned long calculateCRC32(const oem4_binary_header& binhdr, unsigned char* data);

	///Used by CalculateCRC32 function
	unsigned long CRC32Value(int i);

	/**
	 * Reads the serial port for GPS log message and stores the header and data of the message
	 * into the passed arguments
	 * @param binhdr reference to the object of class oem4_binary_header, header data from message stored in this object
	 * @param data pointer to the array that should contain the data of the message, log data from message stored in this array
	 * @return error code
	 */
	int receiveResponse(oem4_binary_header& binhdr, uint8_t* data);

	/**
	 * Parses the array containing the log message and stores all the data in a double vector
	 * @param comdata pointer to the array containing the log message
	 * @return a double vector containing all the data from the log message
	 */
	std::vector<double> parseComData(uint8_t *comdata);

	/**
	 * Responds to the error specified by errorcode
	 * @param errorcode Specifies the error encountered
	 * @return Response to the error
	 */
	int errorResponse(int errorcode);

	const unsigned long CRC32_POLYNOMIAL;
	const int MAXSERIALRECEIVESIZE;

	/// serial port file descriptor
	int fd_ser;
};
#endif
