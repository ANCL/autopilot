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

#include "gx3_read_serial.h"

/* STL Headers */
#include <vector>

/* C headers */
#include <stdint.h>

/* Project Headers */
#include "Debug.h"

void IMU::read_serial::operator()()
{
	uint8_t sync_byte = 0;
	bool found_sync = false;
	std::vector<uint8_t> header(4,0);
	header[0] = 0x75;
	header[1] = 0x65;
	std::vector<uint8_t> buffer;
	buffer.reserve(259); // the gx3 specs a 1 byte length field so this should be enough for max packet length including header
	std::vector<uint8_t> checksum(2, 0);
	const int fd_ser = IMU::getInstance()->fd_ser;
	while (true)
	{
		int bytes = readcond(fd_ser, &sync_byte, 1, 1, 10, 10);
		if (bytes < 1);
		else if (sync_byte == 0x75)
		{
			// got first sync byte
			found_sync = true;
		}
		else if (found_sync && sync_byte == 0x65)
		{
			found_sync = false;
			// got both sync bytes - get message
			uint8_t descriptor = 0, length = 0;
			int bytes = readcond(fd_ser, &descriptor, 1, 1, 10, 10);
			if (bytes < 1)
				continue;
			bytes = readcond(fd_ser, &length, 1, 1, 10 , 10);
			if (bytes < 1)
				continue;
			buffer.resize(length);
			bytes = readcond(fd_ser, &buffer[0], length, length, 10, 10);
			if (bytes < length)
			{
				warning() << "Received valid message header from IMU but did not receive payload.";
				continue;
			}

			bytes = readcond(fd_ser, &checksum[0], 2, 2, 10, 10);
			if (bytes < 2)
				continue;
			// successfully received entire message - compare checksum
			header[2] = descriptor;
			header[3] = length;
			buffer.insert(buffer.begin(), header.begin(), header.end());
			if (checksum != IMU::compute_checksum(buffer))
			{
//				warning() << "IMU checksum failure.  message checksum: " << std::hex << checksum[0] << checksum[1] << "computed checksum: " << ;
				continue;
			}

			// got message, checksum passed, queue it up
			switch (descriptor)
			{
			case COMMAND_BASE:
			case COMMAND_3DM:
			case COMMAND_NAV_FILT:
			case COMMAND_SYS:
			{
//				debug() << "Received command message.";
				IMU::getInstance()->command_queue_push(buffer);
				break;
			}
			case DATA_AHRS:
			{
//				debug() << "Received ahrs message";
				IMU::getInstance()->ahrs_queue_push(buffer);
				break;
			}
			case DATA_GPS:
			{
				debug() << "Received GPS message";
				IMU::getInstance()->gps_queue_push(buffer);
				break;
			}
			case DATA_NAV:
			{
//				debug() << "Got Nav Message";
				IMU::getInstance()->nav_queue_push(buffer);
				break;
			}
			default:
				warning() << "Unknown command received from GX3.  Cannot add it to a queue";
				break;
			}
		}
		else
		{
//			debug() <<"IMU::read_serial: got useless byte";
			found_sync = false;
		}
	}
}
