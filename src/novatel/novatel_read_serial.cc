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

#include "novatel_read_serial.h"

/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* STL Headers */
#include <bitset>

/* C Headers */
#include <termios.h>
#include <math.h>
#include <stdint.h>

/* Boost Headers */
#include <boost/assign.hpp>
// this scope only pollutes the global namespace in a minimal way consistent with the stl global operators
using namespace boost::assign;

/* Project Headers */
//#include "oem4_binary_header.h"
#include "Debug.h"
#include "init_failure.h"

/* read_serial functions */
GPS::read_serial::read_serial()
://CRC32_POLYNOMIAL(0xEDB88320L),
 MAXSERIALRECEIVESIZE(4096)
{
}

GPS::read_serial::read_serial(const read_serial& other)
://CRC32_POLYNOMIAL(0xEDB88320L),
 MAXSERIALRECEIVESIZE(4096)
{
}

void GPS::read_serial::operator()()
{
	debug() << "Initialize the serial port";
	initPort();
	send_log_command();
	readPort();
}

void GPS::read_serial::initPort()
{
	std::string port = "/dev/ser1";
	fd_ser = open(port.c_str(), O_RDWR);
	  if(fd_ser == -1)
	  {
		  critical() << "Unable to open novatel port " << port;
	  }

	  struct termios port_config;

	  tcgetattr(fd_ser, &port_config);                  // get the current port settings
	  cfmakeraw(&port_config);							// set RAW mode
	  cfsetospeed(&port_config, B38400);
	  cfsetispeed(&port_config, B38400);
	  port_config.c_cflag |= (CLOCAL | CREAD);          // Enable the receiver and set local mode...
	  port_config.c_cflag &= ~(CSIZE);                  // Set terminal data length.
	  port_config.c_cflag |=  CS8;                      // 8 data bits
	  port_config.c_cflag &= ~(CSTOPB);                 // clear for one stop bit
	  port_config.c_cflag &= ~(PARENB | PARODD);        // Set terminal parity.
	  // Clear terminal output flow control.
	  port_config.c_iflag &= ~(IXON | IXOFF);           // set -isflow  & -osflow
	  port_config.c_cflag &= ~(IHFLOW | OHFLOW);        // set -ihflow  & -ohflow
	  tcflow (fd_ser, TCION);                           // set -ispaged
	  tcflow (fd_ser, TCOON);                           // set -ospaged
	  tcflow (fd_ser, TCIONHW);                         // set -ihpaged
	  tcflow (fd_ser, TCOONHW);                         // set -ohpaged


	  if (cfsetospeed(&port_config, B38400) != 0)
	    critical() << "could not set output speed";
	  if (cfsetispeed(&port_config, B38400) != 0)
	    critical() << "could not set input speed";
	  if (tcsetattr(fd_ser, TCSADRAIN, &port_config) != 0)
	    critical() << "could not set serial port attributes";
	  tcgetattr(fd_ser, &port_config);
	  tcflush(fd_ser, TCIOFLUSH);

	  //debug() << port << " initialized";
	  //debug() << "Binary Header: " << oem4_binary_header(32, 1);
}


void GPS::read_serial::readPort()
{

	std::vector<uint8_t> buffer;
	buffer.reserve(300);

	std::bitset<2> sync_bytes;

	uint8_t sync_byte = 0;

//	while(!GPS::getInstance()->check_terminate())
	while (true)
	{

		int bytes = readcond(fd_ser, &sync_byte, 1, 1, 10, 10);
		if (bytes < 1);
		else if (sync_byte == 0xAA)
		{
			// got first sync byte
			sync_bytes.reset();
			sync_bytes.set(0);
		}
		else if (sync_bytes[0] && sync_byte == 0x44)
		{
			sync_bytes.set(1);
		}
		else if (sync_bytes.count() == 2 && sync_byte == 0x12)
		{
			sync_bytes.reset();

			int header_size = 25;
			std::vector<uint8_t> header(header_size);
			int bytes = readcond(fd_ser, &header[0], header_size, header_size, 10, 10);
			if (bytes < header_size)
			{
				warning() << "Novatel: Received valid sync bytes, but could not read header";
				continue;
			}

			int data_size = 112;
			std::vector<uint8_t> log_data(data_size);
			bytes = readcond(fd_ser, &log_data[0], data_size, data_size, 10, 10);
			if (bytes < data_size)
			{
				warning() << "Novatel: Received header, but could not receive data log.";
				continue;
			}

			int checksum_size = 4;
			std::vector<uint8_t> checksum(checksum_size);
			bytes = readcond(fd_ser, &checksum[0], checksum_size, checksum_size, 10, 10);
			if (bytes < checksum_size)
			{
				warning() << "Novatel: received log data but could not receive checksum.";
				continue;
			}

			std::vector<uint8_t> whole_message;
			whole_message += 0xAA, 0x44, 0x12;
			whole_message.insert(whole_message.end(), header.begin(), header.end());
			whole_message.insert(whole_message.end(), log_data.begin(), log_data.end());

			if (checksum != compute_checksum(whole_message))
			{
				warning() << "Novatel: received complete message but checksum was invalid";
				continue;
			}

			// test to make sure it is rtkxyz
			if (raw_to_uint16(header.begin() + 1) != 244)
			{
				debug() << "Received message from novatel which wasn't an RTKXYZ message.";
				continue;
			}

			parse_header(header);
//			parse_log(log_data);

		}
		else
		{
			sync_bytes.reset();
		}

//		int errorcode = receiveResponse(binhdr, comdata);
//
//		if(errorcode != OEM4OK)
//		{
//			err_response = errorResponse(errorcode);
//		}
//
//
//		if (binhdr.get_message_type() & 0x80)
//		{
//			uint32_t value_ulong;
//			memcpy(&value_ulong, comdata, sizeof(uint32_t));
//
//			if (value_ulong == 1)
//			{
//				if (binhdr.get_message_id() == 1)
//					debug() << "Log command sent successfully.";
//				else
//					debug() << "Unknown command sent successfully.";
//			}
//			else
//				debug() << "Received error response from GPS";
//
//		}
//
//		gps_output gps_data;  // object to hold current measurement
//		if(err_response == STASHDATA)
//		{
//			critical() << "Received STASHDATA from Novatel GPS";
//			continue;
//		}
//		else if(err_response == BAILOUT)
//		{
//			critical() << "Received BAILOUT from Novatel GPS";
//			continue;
//		}
//		else if(err_response == DONOTHING || err_response == 0)
//		{
//			gps_data = gps_output(parseComData(comdata));
////			LogFile::getInstance()->logData(heli::LOG_NOVATEL_GPS_ALL, gps_data.data());
//		}
//		else
//		{
//			critical() << "Received unknown error code from Novatel GPS: " << errorcode;
//			continue;
//		}
//		GPS::getInstance()->setGPSData(gps_data); // update the raw gps data regardless of mode or validity
//		if (GPS::getInstance()->get_mode() == heli::MODE_GPS_UNINITIALIZED)
//		{
//			if (gps_data.pos_type() == 50)
//				ned_origin_measurements.push_back(gps_data);
//			else
//				ned_origin_measurements.erase(ned_origin_measurements.begin(), ned_origin_measurements.end());
//
//			if (ned_origin_measurements.size() == 5)
//			{
//				GPS::getInstance()->setGPSData(gps_data);
//				double x=0, y=0, z=0;
//				for (int i=0; i<5; ++i)
//				{
//					x += ned_origin_measurements[i].pos_x();
//					y += ned_origin_measurements[i].pos_y();
//					z += ned_origin_measurements[i].pos_z();
//				}
//				x/=5;
//				y/=5;
//				z/=5;
//
//				/* Transformation from ECEF [x,y,z] to geodetic [phi,lamda,h] coordinates using Jay A. Farrel algorithm p.34 */
//				// ECEF2GEO Parameters
//				double lattitude = 0, longitude = 0;
//				double a=6378137.0;
//				double f=1.0/298.257223563;
//				double e=sqrt(f*(2-f));
//
//				// Initialization
//				double h=0;
//				double RN=a;
//				double p=sqrt(pow(x,2)+pow(y,2));
//				double error_h;
//				double prev_h=0;
//				double sin_phi;
//				double phi;
//				double RN_phi;
//
//				// Iteration
//				do
//				{
//					sin_phi=z/((1-pow(e,2))*RN+h);
//					phi=atan((z+pow(e,2)*RN*sin_phi)/p);
//					RN_phi=a/sqrt(1-pow(e,2)*pow(sin(phi),2));
//					h= p/cos(phi)-RN_phi;
//					error_h=h-prev_h;
//					prev_h=h;
//				}
//				while(abs(error_h)>0.000001);
//
//				// Saving results (origin coordinates)
//				lattitude=phi;
//				longitude=atan2(y,x);
//				warning() << "NED Origin set to " << boost::lexical_cast<std::string>(x) << ", " << boost::lexical_cast<std::string>(y) << ", " << boost::lexical_cast<std::string>(z);
//				GPS::getInstance()->set_ned_origin(ned_origin(x,y,z, lattitude, longitude));
//				GPS::getInstance()->set_mode(heli::MODE_GPS_INITIALIZED);
//			}
//		}
//		else
//		{
//			bool pos_valid = false, vel_valid = false;
//
//			if(gps_data.pos_type() < 34.0)
//				GPS::getInstance()->increment_pos_count();
//			else
//			{
//				GPS::getInstance()->reset_pos_count();
//				pos_valid = true;
//			}
//
//			if(gps_data.vel_type() < 34.0)
//				GPS::getInstance()->increment_vel_count();
//			else
//			{
//				GPS::getInstance()->reset_vel_count();
//				vel_valid = true;
//			}
//
//			if (pos_valid && vel_valid) // only update measurement if it's valid!
//			{
//				GPS::getInstance()->set_ned_coords();
//			}
//		}
	}
//
//	debug() << "GPS received terminate, sending 'unlog' command";
//
//	unlog_command_data unlog_data;
//	oem4_binary_header binhd(8, 36);
//
//	sendUnlogCommand(binhd, unlog_data);
}

void GPS::read_serial::parse_header(const std::vector<uint8_t>& header)
{
	std::vector<uint8_t>::const_iterator it = header.begin() + 10;
	uint32_t time_status = raw_to_uint32(it);
	it += 4;
	uint16_t week = raw_to_uint16(it);
	it += 2;
	uint32_t milliseconds = raw_to_uint32(it);
	GPS::getInstance()->set_gps_time(gps_time(week, milliseconds, static_cast<gps_time::TIME_STATUS>(time_status)));
}

void GPS::read_serial::parse_log(const std::vector<uint8_t>& log)
{

}

uint GPS::read_serial::parse_enum(const std::vector<uint8_t>& log, int offset)
{
	return raw_to_uint32(log.begin() + offset);
}



void GPS::read_serial::send_unlog_command()
{
	/* Calculate the CRC.*/
//	unsigned long crc = calculateCRC32(binhd,(unsigned char*)&unlog_data);

	//debug() << "CRC: " << std::hex << crc;

	/* Send out the binary header. */
//	while (write(fd_ser, (unsigned char *)&binhd, binhd.header_size()) < 0)


}

std::vector<uint8_t> GPS::read_serial::generate_header(uint16_t message_id, uint16_t message_length)
{
	std::vector<uint8_t> header;
	header += 0xAA, 0x44, 0x12, 0x00;
	std::vector<uint8_t> length(int_to_raw(message_length));
	header.insert(header.end(), length.begin(), length.end());
	header += 0x00, 192, message_length;
	header.insert(header.end(), 18, 0);
	header[3] = header.size();
	return header;
}

void GPS::read_serial::send_log_command()
{
	std::vector<uint8_t> command(generate_header(244, 32));
	std::vector<uint8_t> port(int_to_raw(6));
	command.insert(command.end(), port.begin(), port.end());
	std::vector<uint8_t> id(int_to_raw(static_cast<uint16_t>(244)));
	command.insert(command.end(), id.begin(), id.end());
	command += 0, 0;
	std::vector<uint8_t> trigger(int_to_raw(2));
	command.insert(command.end(), trigger.begin(), trigger.end());
	std::vector<uint8_t> period(float_to_raw(0.25));
	command.insert(command.end(), period.begin(), period.end());
	command.insert(command.end(), 12, 0);

	std::vector<uint8_t> checksum(compute_checksum(command));
	command.insert(command.end(), checksum.begin(), checksum.end());

	/* Send out the command */
	write(fd_ser, &command[0], command.size());
}

std::vector<uint8_t> GPS::read_serial::compute_checksum(const std::vector<uint8_t>& message)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;

	for (size_t i=0; i < message.size(); i++)
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ message[i] ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}

	return(int_to_raw(ulCRC));
}

unsigned long GPS::read_serial::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}
