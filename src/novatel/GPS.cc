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

#include "GPS.h"

/* Project Headers */
#include "novatel_read_serial.h"

/* C Headers */
#include <math.h>

GPS* GPS::_instance = 0;
boost::mutex GPS::_instance_lock;

GPS* GPS::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new GPS();
	return _instance;
}

GPS::GPS()
: read_serial_thread(read_serial()),
  mode(heli::MODE_GPS_UNINITIALIZED),
  _pos_count(0),
  _vel_count(0),
  _terminate(false)
{
	LogFile::getInstance()->logHeader(heli::LOG_NOVATEL_GPS, "P-sol_status pos_type P-X P-Y P-Z P-X_stddev P-Y_stddev P-Z_stddev "
			"V-sol_status vel_type V-X V-Y V-Z V-X_stddev V-Y_stddev V-Z_stddev "
			"V-latency diff_age sol_age #obs #GPSL1 #L1 #L2");

	MainApp::add_thread(&read_serial_thread, "Novatel GPS");
	MainApp::terminate.connect(GPS::terminate());
}

GPS::~GPS()
{

}
/* terminate functions */
void GPS::terminate::operator()()
{
	GPS* gps = GPS::getInstance();
	boost::mutex::scoped_lock lock(gps->_terminate_lock);
	gps->_terminate = true;
}

/* GPS functions */

bool GPS::pos_is_valid()
{
	boost::mutex::scoped_lock lock(_pos_count_lock);
	return _pos_count < 10;
}

bool GPS::vel_is_valid()
{
	boost::mutex::scoped_lock lock(_vel_count_lock);
	return _vel_count < 10;
}

bool GPS::check_terminate()
{
	boost::mutex::scoped_lock lock(_terminate_lock);
	return _terminate;
}
void GPS::setGPSData(const gps_output& comdata)
{
	boost::mutex::scoped_lock lock(_gps_data_lock);
	_gps_data = comdata;
	LogFile::getInstance()->logData("Novatel GPS", comdata.data());
}

void GPS::set_mode(heli::GPS_MODE mode)
{
	boost::mutex::scoped_lock lock(mode_lock);
	this->mode = mode;
}

heli::GPS_MODE GPS::get_mode()
{
	boost::mutex::scoped_lock lock(mode_lock);
	return this->mode;
}

void GPS::set_ned_origin(const ned_origin& origin)
{
	boost::mutex::scoped_lock lock(_ned_origin_lock);
	_ned_origin = origin;
}

GPS::ned_origin GPS::get_ned_origin()
{
	boost::mutex::scoped_lock lock(_ned_origin_lock);
	return _ned_origin;
}

void GPS::set_ned_coords()
{
	ned_coordinates ned_coords;
	ned_origin origin(get_ned_origin());
	gps_output current_measurement(gps_data());

	// compute xform into ned
	gsl_vector *current_coords=gsl_vector_calloc(3);
	gsl_vector *origin_coords=gsl_vector_calloc(3);

	gsl_vector_set(current_coords,0,current_measurement.pos_x());
	gsl_vector_set(current_coords,1,current_measurement.pos_y());
	gsl_vector_set(current_coords,2,current_measurement.pos_z());
	gsl_vector_set(origin_coords,0,origin.x());
	gsl_vector_set(origin_coords,1,origin.y());
	gsl_vector_set(origin_coords,2,origin.z());

	// Canculating ned coordinates
	gsl_vector_sub(current_coords,origin_coords);
	gsl_vector *gsl_ned_coords = gsl_vector_calloc(3);
    gsl_blas_dgemv(CblasNoTrans, 1.0, origin.ned_rotation(), current_coords, 1.0, gsl_ned_coords);
    ned_coords.set_pos(gsl_ned_coords);

    // Calculating ned velocities
    gsl_vector *velocities=gsl_vector_calloc(3);
    gsl_vector_set(velocities,0,current_measurement.vel_x());
    gsl_vector_set(velocities,1,current_measurement.vel_y());
    gsl_vector_set(velocities,2,current_measurement.vel_z());
    gsl_vector *gsl_ned_velocities=gsl_vector_calloc(3);
    gsl_blas_dgemv(CblasNoTrans, 1.0, origin.ned_rotation(), velocities, 1.0, gsl_ned_velocities);
    ned_coords.set_vel(gsl_ned_velocities);

//    debug() << "Current NED coords: " << ned_coords;

	LogFile::getInstance()->logData("NED Coordinates", (std::vector<double>)ned_coords);

	boost::mutex::scoped_lock lock(_ned_coords_lock);
	_ned_coords = ned_coords;
}

const GPS::ned_coordinates GPS::get_ned_coords()
{
	boost::mutex::scoped_lock lock(_ned_coords_lock);
	return _ned_coords;
}



GPS::gps_output GPS::gps_data()
{
	boost::mutex::scoped_lock lock(_gps_data_lock);
	return _gps_data;
}

GPS::ned_coordinates GPS::ned_coords()
{
	boost::mutex::scoped_lock lock(_ned_coords_lock);
	return _ned_coords;
}




/* globally defined functions */




Debug& operator<<(Debug& dbg, const GPS::ned_coordinates& coord)
{
	dbg << "position = (" << coord.pos_x() << coord.pos_y() << coord.pos_z() << ")";
	dbg << " velocity = (" << coord.vel_x() << coord.vel_y() << coord.vel_z() << ")";
	return dbg;
}


/* ned_coordinates */

GPS::ned_coordinates::ned_coordinates()
{
	position.assign(0);
	velocity.assign(0);
}

GPS::ned_coordinates::operator std::vector<double>()
{
	std::vector<double> coords(position.begin(), position.end());
	coords.insert(coords.end(), velocity.begin(), velocity.end());
	return coords;
}

void GPS::ned_coordinates::set_pos(const gsl_vector* current_coords)
{
	for (int i=0; i<3; i++)
		position[i] = gsl_vector_get(current_coords, i);
}

void GPS::ned_coordinates::set_vel(const gsl_vector* current_velocities)
{
	for (int i=0; i<3; i++)
		velocity[i] = gsl_vector_get(current_velocities, i);
}

GPS::ned_origin::ned_origin(double x, double y, double z, double lattitude, double longitude)
{
	_x = x;
	_y = y;
	_z = z;
	_lattitude = lattitude;
	_longitude = longitude;

	_ned_rotation = gsl_matrix_calloc(3,3);

	/* ECEF to tangent plane rotation matrix, Farrel book p.43 */
	gsl_matrix_set (_ned_rotation, 0, 0, -sin(lattitude)*cos(longitude));
	gsl_matrix_set (_ned_rotation, 0, 1, -sin(lattitude)*sin(longitude));
	gsl_matrix_set (_ned_rotation, 0, 2, cos(lattitude));
	gsl_matrix_set (_ned_rotation, 1, 0, -sin(longitude));
	gsl_matrix_set (_ned_rotation, 1, 1, cos(longitude));
	gsl_matrix_set (_ned_rotation, 1, 2, 0);
	gsl_matrix_set (_ned_rotation, 2, 0, -cos(lattitude)*cos(longitude));
	gsl_matrix_set (_ned_rotation, 2, 1, -cos(lattitude)*sin(longitude));
	gsl_matrix_set (_ned_rotation, 2, 2, -sin(lattitude));
}

GPS::ned_origin::ned_origin(const GPS::ned_origin& other)
{
	_x = other._x;
	_y = other._y;
	_z = other._z;
	_lattitude = other._lattitude;
	_longitude = other._longitude;
	_ned_rotation = gsl_matrix_calloc(3,3);
	gsl_matrix_memcpy(_ned_rotation, other._ned_rotation);
}

const GPS::ned_origin& GPS::ned_origin::operator=(const GPS::ned_origin& other)
{
	if (this == &other)
		return *this;
	_x = other._x;
	_y = other._y;
	_z = other._z;
	_lattitude = other._lattitude;
	_longitude = other._longitude;
	_ned_rotation = gsl_matrix_calloc(3,3);
	gsl_matrix_memcpy(_ned_rotation, other._ned_rotation);
	return *this;
}

GPS::ned_origin::~ned_origin()
{
	gsl_matrix_free(_ned_rotation);
}
