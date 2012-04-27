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

/* C Headers */
#include <termios.h>
#include <math.h>


/* Project Headers */
//#include "oem4_binary_header.h"
#include "Debug.h"

/* read_serial functions */
GPS::read_serial::read_serial()
:CRC32_POLYNOMIAL(0xEDB88320L),
 MAXSERIALRECEIVESIZE(4096)
{
}

GPS::read_serial::read_serial(const read_serial& other)
:CRC32_POLYNOMIAL(0xEDB88320L),
 MAXSERIALRECEIVESIZE(4096)
{
}

void GPS::read_serial::operator()()
{
	debug() << "Initialize the serial port";
	initPort();
	requestData();
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

void GPS::read_serial::requestData()
{
	oem4_binary_header binhdr(32, 1);
	log_command_data log_data;

	sendLogCommand(binhdr, log_data);
}

void GPS::read_serial::readPort()
{
	uint8_t comdata[MAXSERIALRECEIVESIZE];

	std::vector<gps_output> ned_origin_measurements;

	while(!GPS::getInstance()->check_terminate())
	{
		memset(&comdata, 0, MAXSERIALRECEIVESIZE);
		oem4_binary_header binhdr;
		int err_response = 0;

		int errorcode = receiveResponse(binhdr, comdata);

		if(errorcode != OEM4OK)
		{
			err_response = errorResponse(errorcode);
		}


		if (binhdr.get_message_type() & 0x80)
		{
			uint32_t value_ulong;
			memcpy(&value_ulong, comdata, sizeof(uint32_t));

			if (value_ulong == 1)
			{
				if (binhdr.get_message_id() == 1)
					debug() << "Log command sent successfully.";
				else
					debug() << "Unknown command sent successfully.";
			}
			else
				debug() << "Received error response from GPS";

		}

		gps_output gps_data;  // object to hold current measurement
		if(err_response == STASHDATA)
		{
			critical() << "Received STASHDATA from Novatel GPS";
			continue;
		}
		else if(err_response == BAILOUT)
		{
			critical() << "Received BAILOUT from Novatel GPS";
			continue;
		}
		else if(err_response == DONOTHING || err_response == 0)
		{
			gps_data = gps_output(parseComData(comdata));
//			LogFile::getInstance()->logData(heli::LOG_NOVATEL_GPS_ALL, gps_data.data());
		}
		else
		{
			critical() << "Received unknown error code from Novatel GPS: " << errorcode;
			continue;
		}
		GPS::getInstance()->setGPSData(gps_data); // update the raw gps data regardless of mode or validity
		if (GPS::getInstance()->get_mode() == heli::MODE_GPS_UNINITIALIZED)
		{
			if (gps_data.pos_type() == 50)
				ned_origin_measurements.push_back(gps_data);
			else
				ned_origin_measurements.erase(ned_origin_measurements.begin(), ned_origin_measurements.end());

			if (ned_origin_measurements.size() == 5)
			{
				GPS::getInstance()->setGPSData(gps_data);
				double x=0, y=0, z=0;
				for (int i=0; i<5; ++i)
				{
					x += ned_origin_measurements[i].pos_x();
					y += ned_origin_measurements[i].pos_y();
					z += ned_origin_measurements[i].pos_z();
				}
				x/=5;
				y/=5;
				z/=5;

				/* Transformation from ECEF [x,y,z] to geodetic [phi,lamda,h] coordinates using Jay A. Farrel algorithm p.34 */
				// ECEF2GEO Parameters
				double lattitude = 0, longitude = 0;
				double a=6378137.0;
				double f=1.0/298.257223563;
				double e=sqrt(f*(2-f));

				// Initialization
				double h=0;
				double RN=a;
				double p=sqrt(pow(x,2)+pow(y,2));
				double error_h;
				double prev_h=0;
				double sin_phi;
				double phi;
				double RN_phi;

				// Iteration
				do
				{
					sin_phi=z/((1-pow(e,2))*RN+h);
					phi=atan((z+pow(e,2)*RN*sin_phi)/p);
					RN_phi=a/sqrt(1-pow(e,2)*pow(sin(phi),2));
					h= p/cos(phi)-RN_phi;
					error_h=h-prev_h;
					prev_h=h;
				}
				while(abs(error_h)>0.000001);

				// Saving results (origin coordinates)
				lattitude=phi;
				longitude=atan2(y,x);
				warning() << "NED Origin set to " << boost::lexical_cast<std::string>(x) << ", " << boost::lexical_cast<std::string>(y) << ", " << boost::lexical_cast<std::string>(z);
				GPS::getInstance()->set_ned_origin(ned_origin(x,y,z, lattitude, longitude));
				GPS::getInstance()->set_mode(heli::MODE_GPS_INITIALIZED);
			}
		}
		else
		{
			bool pos_valid = false, vel_valid = false;

			if(gps_data.pos_type() < 34.0)
				GPS::getInstance()->increment_pos_count();
			else
			{
				GPS::getInstance()->reset_pos_count();
				pos_valid = true;
			}

			if(gps_data.vel_type() < 34.0)
				GPS::getInstance()->increment_vel_count();
			else
			{
				GPS::getInstance()->reset_vel_count();
				vel_valid = true;
			}

			if (pos_valid && vel_valid) // only update measurement if it's valid!
			{
				GPS::getInstance()->set_ned_coords();
			}
		}
	}

	debug() << "GPS received terminate, sending 'unlog' command";

	unlog_command_data unlog_data;
	oem4_binary_header binhd(8, 36);

	sendUnlogCommand(binhd, unlog_data);
}

std::vector<double> GPS::read_serial::parseComData(uint8_t *comdata)
{
	std::vector<double> com_data(23);

	uint32_t enum_type = 0;
	memcpy(&enum_type, comdata, 4);
	com_data[0] = static_cast<double>(enum_type);				//Position solution status

//	debug() << "Pos sol status: " << enum_type;

	memcpy(&enum_type, comdata+4, 4);
	com_data[1] = static_cast<double>(enum_type);				//Position type

//	debug() << "Pos type: " << enum_type;

	double P = 0;
	memcpy(&P, comdata+8, 8);											//Position X Coordinate
	com_data[2] = P;

	memcpy(&P, comdata+16, 8);
	com_data[3] = P;											//Position Y Coordinate

//	debug() << "Pos y: " << P;

	memcpy(&P, comdata+24, 8);
	com_data[4] = P;											//Position Z Coordinate

//	debug() << "Pos z: " << P;

	float stddev = 0;
	memcpy(&stddev, comdata+32, 4);
	com_data[5] = static_cast<double>(stddev);					//Standard deviation of X Coordinate position

	memcpy(&stddev, comdata+36, 4);
	com_data[6] = static_cast<double>(stddev);					//Standard deviation of Y Coordinate position

	memcpy(&stddev, comdata+40, 4);
	com_data[7] = static_cast<double>(stddev);					//Standard deviation of Z Coordinate position

	memcpy(&enum_type, comdata+44, 4);
	com_data[8] = static_cast<double>(enum_type);				//Velocity solution status

	memcpy(&enum_type, comdata+48, 4);
	com_data[9] = static_cast<double>(enum_type);				//Velocity type

	memcpy(&P, comdata+52, 8);
	com_data[10] = P;											//Velocity X Coordinate

	memcpy(&P, comdata+60, 8);
	com_data[11] = P;											//Velocity Y Coordinate

	memcpy(&P, comdata+68, 8);
	com_data[12] = P;											//Velocity Z Coordinate

	memcpy(&stddev, comdata+76, 4);
	com_data[13] = static_cast<double>(stddev);					//Standard deviation of X Coordinate velocity

	memcpy(&stddev, comdata+80, 4);
	com_data[14] = static_cast<double>(stddev);					//Standard deviation of Y Coordinate velocity

	memcpy(&stddev, comdata+84, 4);
	com_data[15] = static_cast<double>(stddev);					//Standard deviation of Z Coordinate velocity

	memcpy(&stddev, comdata+92, 4);
	com_data[16] = static_cast<double>(stddev);					//V-Latency

	memcpy(&stddev, comdata+96, 4);
	com_data[17] = static_cast<double>(stddev);					//Differential age in seconds

	memcpy(&stddev, comdata+100, 4);
	com_data[18] = static_cast<double>(stddev);					//Solution age

	uint8_t char_type = 0;
	memcpy(&char_type, comdata+104, 1);
	com_data[19] = static_cast<double>(char_type);				//Number of observations tracked

	memcpy(&char_type, comdata+105, 1);
	com_data[20] = static_cast<double>(char_type);				//Number of GPS L1 ranges used in computation

	memcpy(&char_type, comdata+106, 1);
	com_data[21] = static_cast<double>(char_type);				//Number of GPS  L1 ranges above the RTK mask angle

	memcpy(&char_type, comdata+107, 1);
	com_data[22] = static_cast<double>(char_type);				//Number of GPS L2 ranges above the RTK mask angle

	return com_data;
}

int GPS::read_serial::receiveResponse(oem4_binary_header& binhdr, uint8_t* comdata)
{
	int numbytes = readcond(fd_ser, (uint8_t*)&binhdr, binhdr.header_size(), binhdr.header_size(), 10, 10);

//		debug() << "Received Header: " << std::endl << binhdr;

	if (numbytes < 0)
	{
		/* Really bad error. */
		return OEM4ERROR_CATASTROPHIC;
	}

	if (numbytes < 28)
	{
		return OEM4ERROR_TIMEOUT_HEADER;
	}

	numbytes = readcond(fd_ser, (uint8_t*)comdata, binhdr.message_size(), binhdr.message_size(), 0, 10);

	if (numbytes < binhdr.message_size())
	{
		if (numbytes < 0)
		{
			/* Really bad error. */
			return OEM4ERROR_CATASTROPHIC;
		}
		else
		{
			return OEM4ERROR_TIMEOUT_DATA;
		}
	}

	unsigned long crc_received;
	numbytes = readcond(fd_ser, (unsigned char *) &crc_received, 4, 4, 0, 10);

	if (numbytes < 0)
	{
		/* Really bad error. */
		return OEM4ERROR_CATASTROPHIC;
	}

	if (numbytes < 4)
	{
		return OEM4ERROR_TIMEOUT_CRC;
	}

	unsigned long crc_calculated = calculateCRC32(binhdr, (uint8_t*)comdata);

	if (crc_calculated != crc_received)
	{
		/* CRC Mismatch. */
		return OEM4ERROR_CRC_MISMATCH;
	}

	//debug() << "CRC Check successful";

	return OEM4OK;
}

int GPS::read_serial::errorResponse(int errorcode)
{
	switch(errorcode)
	{
	case OEM4ERROR_TIMEOUT_HEADER:
	{
		debug() << __FILE__ << __LINE__ << "Could not receive message header, timed out";
		return STASHDATA;
		break;
	}

	case OEM4ERROR_TIMEOUT_DATA:
	{
		debug() << __FILE__ << __LINE__ << "Could not receive log data, timed out";
		return STASHDATA;
		break;
	}

	case OEM4ERROR_TIMEOUT_CRC:
	{
		debug() << __FILE__ << __LINE__ << "Could not receive checksum bits, timed out";
		return STASHDATA;
		break;
	}

	case OEM4ERROR_CRC_MISMATCH:
	{
		debug() << __FILE__ << __LINE__ << "Checksum mismatch. Data corrupted";
		return STASHDATA;
		break;
	}

	case OEM4ERROR_MISSING_INPUT_ARGS:
	{
		debug() << __FILE__ << __LINE__ << "Missing input arguments";
		break;
	}

	case OEM4ERROR_MISSING_OUTPUT_SPACE:
	{
		debug() << __FILE__ << __LINE__ << "Missing output space";
		break;
	}

	case OEM4ERROR_NOT_RESPONSE:
	{
		debug() << __FILE__ << __LINE__ << "Message received not a response";
		break;
	}

	case OEM4ERROR_COMMAND_FAILED:
	{
		debug() << __FILE__ << __LINE__ << "Command failed";
		break;
	}

	case OEM4ERROR_CATASTROPHIC:
	{
		critical() << __FILE__ << __LINE__ << "Catastrophic error received, bailing out";
		return BAILOUT;
		break;
	}

	default:
		debug() << __FILE__ << __LINE__ << "Unknown error code received";
		break;
	}
	return -1;
}

void GPS::read_serial::sendUnlogCommand (const oem4_binary_header &binhd, const unlog_command_data& unlog_data)
{
	/* Calculate the CRC.*/
	unsigned long crc = calculateCRC32(binhd,(unsigned char*)&unlog_data);

	//debug() << "CRC: " << std::hex << crc;

	/* Send out the binary header. */
	while (write(fd_ser, (unsigned char *)&binhd, binhd.header_size()) < 0)
	{
		/* Error.  Repeat.*/
		debug() << "GPS error on bin header transmit: command " << binhd.get_message_id() << " Retrying.";
	}

	/* Send out all data but the CRC. */
	while (write(fd_ser, (unsigned char *)&unlog_data, binhd.message_size()) < 0)
	{
		/* Error.  Repeat.*/
		debug() << "GPS error on data transmit: command " << binhd.get_message_id() << "Retrying.";
	}

	/* Send out the CRC. */
	while (write(fd_ser, &crc, sizeof(unsigned long)) < 0)
	{
		/* Error.  Repeat.*/
		debug() << "DGPS error on CRC send: command " << binhd.get_message_id() << "Retrying.";
	}


}

void GPS::read_serial::sendLogCommand (const oem4_binary_header &binhdr, const log_command_data& log_data)
{
	//debug() << "Binary Header: " << binhdr;
	//debug() << "Log Command: " << log_data;

	/* Calculate the CRC.*/
	unsigned long crc = calculateCRC32(binhdr,(unsigned char*)&log_data);

	//debug() << "CRC: " << std::hex << crc;

	/* Send out the binary header. */
	while (write(fd_ser, (unsigned char *)&binhdr, binhdr.header_size()) < 0)
	{
		/* Error.  Repeat.*/
		debug() << "GPS error on bin header transmit: command " << binhdr.get_message_id() << " Retrying.";
	}

	/* Send out all data but the CRC. */
	while (write(fd_ser, (unsigned char *)&log_data, binhdr.message_size()) < 0)
	{
		/* Error.  Repeat.*/
		debug() << "GPS error on data transmit: command " << binhdr.get_message_id() << "Retrying.";
	}

	/* Send out the CRC. */
	while (write(fd_ser, &crc, sizeof(unsigned long)) < 0)
	{
		/* Error.  Repeat.*/
		debug() << "DGPS error on CRC send: command " << binhdr.get_message_id() << "Retrying.";
	}

}

unsigned long GPS::read_serial::calculateCRC32(const oem4_binary_header& binheader, unsigned char *data)
{
	/* Deal with the binary header. */
	unsigned char *ucBuffer = (unsigned char *)&binheader;

	unsigned char ulCount = binheader.header_size();

	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( ulCount-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}

	/* Deal with the data. */
	ucBuffer = data;
	ulCount = binheader.message_size();
	while ( ulCount-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}

	return( ulCRC );
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




/* log_command_data functions */

GPS::read_serial::log_command_data::log_command_data()
{
	port = THISPORT;	        // THISPORT
	msgID = 244;				// RTKXYZ
	msgtype = 0;				// binary
	reserved = 0;			// unused
	trigger = 2;				// ONTIME
	period = 0.1;			// 0.1 seconds (10 Hz)
	offset = 0;				// no offset
	hold = 0;				// allow log to be removed by UNLOGALL
}

/* gps_output functions */

GPS::gps_output::gps_output(std::vector<double> com_data)
{
	this->com_data = com_data;
}

/* unlog_command_data functions */
GPS::read_serial::unlog_command_data::unlog_command_data()
{
	port = THISPORT;		//THISPORT
	msgId = 244;			//RTKXYZ
	msgtype = 0; 			//binary
	reserved = 0;			//unused
}

Debug& operator<<(Debug& dbg, const GPS::read_serial::log_command_data& logcom)
{
	dbg<< "Period:" << logcom.period << std::endl;

	return dbg;
}
