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
#include <boost/thread.hpp>

/* Project Headers */
//#include "oem4_binary_header.h"
#include "Debug.h"
#include "init_failure.h"

/* read_serial functions */
GPS::read_serial::read_serial()

{
}

void GPS::read_serial::operator()()
{
	debug() << "Initialize the serial port";
	initPort();
	send_log_command();
	readPort();
	send_unlog_command();
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

	// todo add in check terminate
//	while(!GPS::getInstance()->check_terminate())
	while (true)
	{
		if ((boost::posix_time::microsec_clock::local_time() - last_data).total_milliseconds > 3)
		{
			send_unlog_command();
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			send_log_command();
		}

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

			int data_size = raw_to_int<uint16_t>(header.begin() + 5);
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

			uint16_t message_id = raw_to_int<uint16_t>(header.begin() + 1);

			switch (message_id)
			{
			case 244:  // RTKXYZ
			{
				if (is_response(header))
				{
					switch(parse_enum(log_data))
					{
					case OEM4OK:
						message() << "Novatel data logging successfully initialized";
						break;
					case OEM4_CRC_MISMATCH:
						warning() << "Novatel reports checksum failure";
						break;
					}
				}
				else
				{
					std::vector<double> log;
					parse_header(header, log);
					parse_log(log_data, log);
					last_data = boost::posix_time::microsec_clock::local_time();
				}
				break;
			}
			default:
			{
				warning() << "Received unexpected message from Novatel with id: " << message_id;
				continue;
			}
			}

		}
		else
		{
			sync_bytes.reset();
		}
	}
//
//	debug() << "GPS received terminate, sending 'unlog' command";
//
//	unlog_command_data unlog_data;
//	oem4_binary_header binhd(8, 36);
//
//	sendUnlogCommand(binhd, unlog_data);
}

bool GPS::read_serial::is_response(const std::vector<uint8_t>& header)
{
	std::bitset<8> message_type(header[3]);
	return message_type[7];
}

void GPS::read_serial::parse_header(const std::vector<uint8_t>& header, std::vector<double>& log)
{
	std::vector<uint8_t>::const_iterator it = header.begin() + 10;
	uint32_t time_status = parse_enum(header, 10);
	log += time_status;
	it += 4;
	uint16_t week = raw_to_int<uint16_t>(it);
	log += week;
	it += 2;
	uint32_t milliseconds = raw_to_int<uint32_t>(it);
	log += milliseconds;
	GPS::getInstance()->set_gps_time(gps_time(week, milliseconds, static_cast<gps_time::TIME_STATUS>(time_status)));
}

void GPS::read_serial::parse_log(const std::vector<uint8_t>& data, std::vector<double>& log)
{
	GPS& gps = *GPS::getInstance();

	uint pos_status = parse_enum(data);
	log += pos_status;
	gps.set_position_status(pos_status);

	uint pos_type = parse_enum(data, 4);
	log += pos_type;
	gps.set_position_type(pos_type);

	blas::vector<double> position(parse_3floats<double>(data, 8));
	log.insert(log.end(), position.begin(), position.end());
	blas::vector<double> llh(ecef_to_llh(position));
	gps.set_llh_position(llh);

	blas::vector<float> position_error(parse_3floats<float>(data, 32));
	log.insert(log.end(), position_error.begin(), position_error.end());
	gps.set_pos_sigma(ecef_to_ned(position_error, llh));

	uint vel_status = parse_enum(data, 44);
	log += vel_status;
	gps.set_velocity_status(vel_status);

	uint vel_type = parse_enum(data, 48);
	log += vel_type;
	gps.set_velocity_type(vel_type);

	blas::vector<double> velocity(parse_3floats<double>(data, 52));
	log.insert(log.end(), velocity.begin(), velocity.end());
	gps.set_ned_velocity(ecef_to_ned(velocity, llh));

	blas::vector<float> velocity_error(parse_3floats<float>(data, 76));
	log.insert(log.end(), velocity_error.begin(), velocity_error.end());
	gps.set_vel_sigma(ecef_to_ned(velocity_error, llh));

	uint8_t num_sats = log[104];
	log += num_sats;
	gps.set_num_sats(num_sats);
}

uint GPS::read_serial::parse_enum(const std::vector<uint8_t>& log, int offset)
{
	return raw_to_int<uint32_t>(log.begin() + offset);
}

blas::vector<double> GPS::read_serial::ecef_to_llh(const blas::vector<double>& ecef)
{
	/* Transformation from ECEF [x,y,z] to geodetic [phi,lamda,h] coordinates using Jay A. Farrel algorithm p.34 */
	// ECEF2GEO Parameters
	blas::vector<double> llh(3);
	llh.clear();

	double a = 6378137.0;
	double f = 1.0/298.257223563;
	double e = sqrt(f*(2-f));

	// Initialization
	double RN = a;
	double p=sqrt(pow(ecef[0],2)+pow(ecef[1],2));
	double error_h = 0;
	double prev_h = 0;
	double sin_phi = 0;
	double phi = 0;
	double RN_phi = 0;

	// Iteration
	do
	{
		sin_phi = ecef[2]/((1 - pow(e,2))*RN + llh[2]);
		phi = atan((ecef[2] + pow(e,2)*RN*sin_phi)/p);
		RN_phi = a/sqrt(1 - pow(e,2)*pow(sin(phi),2));
		llh[2] = p/cos(phi)-RN_phi;
		error_h = llh[2] - prev_h;
		prev_h = llh[2];
	}
	while(abs(error_h)>0.000001);

	// Saving results (origin coordinates)
	llh[0] = phi; //latitude
	llh[1] = atan2(ecef[1], ecef[0]); //longitude

	return llh;
}




void GPS::read_serial::send_unlog_command()
{
	// todo fill in this function
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
