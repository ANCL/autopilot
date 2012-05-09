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

#include "IMU.h"

/* Project Headers */
#include "Debug.h"
#include "init_failure.h"
#include "gx3_read_serial.h"
#include "MainApp.h"
#include "message_parser.h"
#include "ack_handler.h"
#include "gx3_send_serial.h"
#include "QGCLink.h"

/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* C Headers */
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstdlib>
#include <math.h>

/* Boost Headers */
#include <boost/math/constants/constants.hpp>

IMU* IMU::_instance = NULL;
boost::mutex IMU::_instance_lock;
const std::string IMU::serial_port = "/dev/ser2";

IMU* IMU::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new IMU;
	return _instance;
}

IMU::IMU()
:
 position(blas::zero_vector<double>(3)),
 ned_origin(blas::zero_vector<double>(3)),
 velocity(blas::zero_vector<double>(3)),
 use_nav_attitude(true),
 attitude_source_connection(QGCLink::getInstance()->attitude_source.connect(
		 boost::bind(&IMU::set_use_nav_attitude, this, _1))),
 nav_euler(blas::zero_vector<double>(3)),
 ahrs_euler(blas::zero_vector<double>(3)),
 nav_rotation(blas::identity_matrix<double>(3)),
 nav_angular_rate(blas::zero_vector<double>(3)),
 ahrs_angular_rate(blas::zero_vector<double>(3))
{

	try
	{
		init_serial();
	}
	catch (init_failure i)
	{
		critical() << "Failed to open serial port for GX3.  Attempting to terminate autopilot.";
		MainApp::terminate();
	}

	receive_thread = boost::thread(read_serial());
	parse_thread = boost::thread(message_parser());
	_send_serial.reset(new send_serial(this));
}

IMU::~IMU()
{
	close_serial();
}

void IMU::init_serial()
{
	fd_ser = open(serial_port.c_str(), O_RDWR);
	  if(fd_ser == -1)
	      throw init_failure("Could not initialize imu serial port");

	  struct termios port_config;

	  tcgetattr(fd_ser, &port_config);                  // get the current port settings
	  cfmakeraw(&port_config);							// set RAW mode
	  cfsetospeed(&port_config, B115200);
	  cfsetispeed(&port_config, B115200);
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

	  if (cfsetospeed(&port_config, B115200) != 0)
	    critical() << "could not set output speed";
	  if (cfsetispeed(&port_config, B115200) != 0)
	    critical() << "could not set input speed";
	  if (tcsetattr(fd_ser, TCSADRAIN, &port_config) != 0)
	    critical() << "could not set serial port attributes";
	  tcgetattr(fd_ser, &port_config);
	  tcflush(fd_ser, TCIOFLUSH);
}

void IMU::close_serial()
{
	close(fd_ser);
}

std::vector<uint8_t> IMU::compute_checksum(std::vector<uint8_t> data)
{
	std::vector<uint8_t> checksum(2,0);
	for (std::vector<uint8_t>::const_iterator it = data.begin(); it != data.end(); ++it)
	{
		checksum[0] += *it;
		checksum[1] += checksum[0];
	}
	return checksum;
}

void IMU::set_gx3_mode(IMU::GX3_MODE mode)
{
	bool mode_changed = false;
	{
		boost::mutex::scoped_lock lock(gx3_mode_lock);
		if (mode != gx3_mode)
		{
			if (mode == RUNNING && gx3_mode != ERROR) // just came out of init or startup
				set_ned_origin();
			gx3_mode = mode;
			mode_changed = true;
		}

	}
	if (mode_changed)
		gx3_mode_changed(mode);
}

//blas::vector<double> IMU::get_euler() const
//{
//	blas::matrix<double> rot(blas::trans(get_rotation()));
//	blas::vector<double> euler(3);
//	euler.clear();
//	// roll
//	euler[0] = ::atan2(rot(2,1), rot(2,2));
//	// pitch
//	euler[1] = -::atan(rot(2,0)/::sqrt(1-pow(rot(2,0),2)));
//	// yaw
//	euler[2] = ::atan2(rot(1,0), rot(0,0));
//
//	return euler;
//}

blas::vector<double> IMU::get_euler_rate() const
{
//	blas::vector<double> euler(get_euler());
//	double roll = euler[0];
//	double pitch = euler[1];
//	blas::matrix<double> xform(3,3);
//	xform.clear();
//	xform(0,0) = 1;
//	xform(0,1) = sin(roll)*tan(pitch);
//	xform(0,2) = cos(roll)*tan(pitch);
//	xform(1,1) = cos(roll);
//	xform(1,2) = -sin(roll);
//	xform(2,1) = sin(roll)/cos(pitch);
//	xform(2,2) = cos(roll)/cos(pitch);

//	return prod(xform, get_angular_rate());

	// just return the angular rate since the yaw gyro measurement is not reliable
	return get_angular_rate();
}

blas::vector<double> IMU::get_ned_position() const
{
	blas::vector<double> llh(get_llh_position());
	llh[0]*=boost::math::constants::pi<double>()/180;
	llh[1]*=boost::math::constants::pi<double>()/180;

	blas::matrix<double> rot(3,3);
	rot.clear();
	rot(0,0) = -sin(llh[0])*cos(llh[1]);
	rot(0,1) = -sin(llh[0])*sin(llh[1]);
	rot(0,2) = cos(llh[0]);
	rot(1,0) = -sin(llh[1]);
	rot(1,1) = cos(llh[1]);
	rot(2,0) = -cos(llh[0])*cos(llh[1]);
	rot(2,1) = -cos(llh[0])*sin(llh[1]);
	rot(2,2) = -sin(llh[0]);

	return prod(rot, get_ecef_position() - get_ecef_origin());  // take advantage of matrix expression type
}

blas::vector<double> IMU::llh2ecef(const blas::vector<double>& llh_deg)
{
	// wgs84 constants
	blas::vector<double> llh(llh_deg);
	llh[0]*=boost::math::constants::pi<double>()/180;
	llh[1]*=boost::math::constants::pi<double>()/180;
	static const double equatorial_radius = 6378137;
	static const double flatness = 1/298.257223563;
	static double eccentricity = sqrt(flatness*(2-flatness));

	double normal_radius = equatorial_radius/sqrt(1 - pow(eccentricity, 2)*pow(sin(llh[0]),2));

	blas::vector<double> ecef(3);
	ecef.clear();

	ecef[0] = (normal_radius + llh[2])*cos(llh[0])*cos(llh[1]);
	ecef[1] = (normal_radius + llh[2])*cos(llh[0])*sin(llh[1]);
	ecef[2] = (normal_radius*(1-pow(eccentricity,2)) + llh[2])*sin(llh[0]);

	return ecef;
}

void IMU::set_use_nav_attitude(bool attitude_source)
{
	boost::mutex::scoped_lock lock(use_nav_attitude_lock);
	use_nav_attitude = attitude_source;
	message() << "Attitude source changed to " << (attitude_source?"nav filter":"ahrs") << ".";
}

blas::matrix<double> IMU::euler_to_rotation(const blas::vector<double>& euler)
{
	blas::matrix<double> rot(3,3);
	rot.clear();

	double roll = euler[0], pitch = euler[1], yaw = euler[2];
	rot(0, 0) = cos(yaw)*cos(pitch);
	rot(0, 1) = sin(yaw)*cos(pitch);
	rot(0, 2) = -sin(pitch);
	rot(1, 0) = -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll);
	rot(1, 1) = cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll);
	rot(1, 2) = cos(pitch)*sin(roll);
	rot(2, 0) = sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll);
	rot(2, 1) = -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
	rot(2, 2) = cos(pitch)*cos(roll);

	return rot;
}

void IMU::set_ned_origin(const blas::vector<double>& origin)
{
	{
	boost::mutex::scoped_lock lock(ned_origin_lock);
	ned_origin = origin;
	}
	message() << "Origin set to: " << origin;
}
