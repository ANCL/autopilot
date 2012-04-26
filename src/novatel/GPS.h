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

#ifndef GPS_H_
#define GPS_H_

/* GSL Headers */
#include <gsl/gsl_matrix.h>

/* Project Headers */
#include "Debug.h"

/**
 * @brief Read position and velocity measurements from the Novatel GPS.
 *
 * This class performs the communication with the novatel gps.  It initiates logging of message number 244, RTKXYZ RTK Cartesian Position and Velocity
 * a description of which can be found on page 291 of the OEM4 Manual vol 2, Command and Log Reference.
 *
 * The GPS is connected to /dev/ser2 and uses 38400 baud, no parity, 8 data bits, 1 stop bit, no flow control.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Aakash Vasudevan <avasudev@ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 *
 * @date January 2012: Class creation and initial implementation
 * @date February 16, 2012: Refactored into separate files
 */

class GPS
{
public:
	virtual ~GPS();

	static GPS* getInstance();

	/**
	 * Checks if the current GPS position data is valid (Invalid if the past 5 successive GPS position data type is less than 34, otherwise valid)
	 * The variable to track the number of successive data with GPS position type less than 34 is incremented in GPS::read_serial_read_port().
	 * @return true if valid, otherwise false
	 */
	bool pos_is_valid();

	/**
	 * Checks if the current velocity GPS data is valid (Invalid if the past 5 successive GPS velocity data type is less than 34, otherwise valid)
	 * The variable to track the number of successive data with GPS velocity type less than 34 is incremented in GPS::read_serial_read_port().
	 * @return true if valid, otherwise false
	 */
	bool vel_is_valid();


	class read_serial;




	/** @brief This class contains the output of the novatel gps
	 * The output is stored in a double vector that can be accessed using
	 * accessors defined in the class.
	 */
	class gps_output
	{
	public:
		explicit gps_output(std::vector<double> com_data = std::vector<double>(22));

		/* From page 291 of Novatel documentation vol.2 */
		/* Accessors */
		double pos_x() {return com_data[2];}
		double pos_y() {return com_data[3];}
		double pos_z() {return com_data[4];}

		double pos_stddev_x() {return com_data[5];}
		double pos_stddev_y() {return com_data[6];}
		double pos_stddev_z() {return com_data[7];}

		double vel_x() {return com_data[10];}
		double vel_y() {return com_data[11];}
		double vel_z() {return com_data[12];}

		double vel_stddev_x() {return com_data[13];}
		double vel_stddev_y() {return com_data[14];}
		double vel_stddev_z() {return com_data[15];}

		/// @returns position solution type
		double pos_type() {return com_data[1];}
		/// @returns velocity solution type
		double vel_type() {return com_data[9];}

		/// @returns the data as a vector (intended for logging)
		inline const std::vector<double>& data() const {return com_data;}

	private:
		std::vector<double> com_data;

	};

	class ned_coordinates
	{
	public:
		ned_coordinates();
		double pos_x() const {return position[0];}
		double pos_y() const {return position[1];}
		double pos_z() const {return position[2];}


		double vel_x() const {return velocity[0];}
		double vel_y() const {return velocity[1];}
		double vel_z() const {return velocity[2];}

		void set_pos(const gsl_vector* current_coords);
		void set_vel(const gsl_vector* current_velocities);

		operator std::vector<double>();
		const boost::array<double, 3>& getPosition() const {return position;}
		const boost::array<double, 3>& getVelocity() const {return velocity;}

		friend Debug& operator<<(Debug& dbg, const ned_coordinates& coord);
	private:
		boost::array<double, 3> position;
		boost::array<double, 3> velocity;
	};

	class ned_origin
	{
	public:
		explicit ned_origin( double x=0, double y=0, double z=0, double lattitude=0, double longitude=0);
		ned_origin(const ned_origin& other);
		virtual ~ned_origin();
		const double& x() const {return _x;}
		double& x() {return _x;}
		const double& y() const {return _y;}
		double& y() {return _y;}
		const double& z() const {return _z;}
		double& z() {return _z;}
		const double& longitude() const {return _longitude;}
		double& longitude() {return _longitude;}
		const double& lattitude() const {return _lattitude;}
		double& lattitude() {return _lattitude;}
		const gsl_matrix* ned_rotation() const {return _ned_rotation;}
		const ned_origin& operator=(const ned_origin& other);
	private:
		double _x;
		double _y;
		double _z;
		double _longitude;
		double _lattitude;

		gsl_matrix *_ned_rotation;
	};

	/// returns a copy of the current gps measurement see GPS::_gps_data
	gps_output gps_data();

	ned_coordinates ned_coords();

	/// Returns the current ned coordinates representation for the current gps measurement see GPS::_ned_coords
	const ned_coordinates get_ned_coords();

private:

	/// default constructor
	GPS();
	/// pointer to <em> the </em> instance of GPS
	static GPS* _instance;
	/// serialize access to _instance
	static boost::mutex _instance_lock;

	/// thread used to communicate with the GPS
	boost::thread read_serial_thread;

	/**
	 * The data member stores the current gps measurement.  This measurement is always updated even if the solution type is not valid.
	 * Therefore this value should not be used as feedback for a controller.  For a value suitable for feedback see GPS::_ned_coords
	 */
	gps_output _gps_data;

	/// serialize access to gps_data
	boost::mutex _gps_data_lock;

	/**
	 * This member contains the current GPS mesurement in NED coordinates.  Unlike GPS::_gps_data it is updated only when both the
	 * position and velocity solution types are greater than or equal to 34.
	 */
	ned_coordinates _ned_coords;

	/// serialize access to _ned_coords
	boost::mutex _ned_coords_lock;

	/// computes the current ned coordinate representation for the current gps measurement
	void set_ned_coords();


	ned_origin _ned_origin;
	void set_ned_origin(const ned_origin& origin);
	ned_origin get_ned_origin();
	boost::mutex _ned_origin_lock;

	heli::GPS_MODE mode;
	boost::mutex mode_lock;

	heli::GPS_MODE get_mode();
	void set_mode(heli::GPS_MODE mode);

	///Number of consecutive data received with position type less than 34
	int _pos_count;
	///Makes access to _pos_count threadsafe
	boost::mutex _pos_count_lock;

	inline void increment_pos_count() {boost::mutex::scoped_lock lock(_pos_count_lock); _pos_count++;}
	inline void reset_pos_count() {boost::mutex::scoped_lock lock(_pos_count_lock); _pos_count = 0;}

	///Number of consecutive data received with velocity type less than 34
	int _vel_count;
	///Makes access to _vel_count threadsafe
	boost::mutex _vel_count_lock;

	inline void increment_vel_count() {boost::mutex::scoped_lock lock(_vel_count_lock); _vel_count++;}
	inline void reset_vel_count() {boost::mutex::scoped_lock lock(_vel_count_lock); _vel_count = 0;}

	/**
	 * Update the GPS data with the new measurement. This function also
	 * logs the new value.
	 * @param comdata double vector of data from GPS
	 */
//	void setGPSData(const std::vector<double>& comdata);
	void setGPSData(const gps_output& comdata);

	bool _terminate;
	boost::mutex _terminate_lock;
	bool check_terminate();

	/**
	 * @brief Class to terminate communication with GPS. Called when SIGINT is received.
	 */
	class terminate
	{
	public:
		void operator()();
	};

	/// convert a raw string of bytes received from the novatel to an unsigned integer
	template <typename ReturnType, typename IteratorType>
	static ReturnType raw_to_uint(IteratorType first, IteratorType last);

	/// convert a novatel long type to an integer
	template <typename IteratorType>
	static inline uint32_t raw_to_uint32(IteratorType first) {return raw_to_uint<uint32_t>(first, first + 4);}

	/// convert a novatel short type to an integer
	template <typename IteratorType>
	static inline uint16_t raw_to_uint16(IteratorType first) {return raw_to_uint<uint16_t>(first, first + 2);}

	/// convert a raw string of bytes to a signed integer
	template <typename ReturnType, typename IteratorType>
	static ReturnType raw_to_int(IteratorType first, IteratorType last);

	template <typename IteratorType>
	static inline int32_t raw_to_int32(IteratorType first) {return raw_to_int<int32_t>(first, first + 4);}

	template <typename IteratorType>
	static inline int16_t raw_to_int16(IteratorType first) {return raw_to_int<int16_t>(first, first + 2);}

	/// convert an integer type (signed or unsigned) to raw
	template <typename IntegerType>
	static std::vector<uint8_t> int_to_raw(const IntegerType i);
};


template <typename ReturnType, typename IteratorType>
ReturnType GPS::raw_to_uint(IteratorType first, IteratorType last)
{
	unsigned int data = 0; // uint is largest int type from novatel
	for (IteratorType it = first; it != last; ++it)
	{
		data += *it;
		data <<= 8;
	}

	switch (last - first)
	{
	case 1:
		return static_cast<uint8_t>(data);
		break;
	case 2:
		return static_cast<uint16_t>(data);
		break;
	default:
		return data;
	}
}

template <typename ReturnType, typename IteratorType>
ReturnType GPS::raw_to_int(IteratorType first, IteratorType last)
{
	int32_t data = 0; // int is largest int type from novatel
	for (IteratorType it = first; it != last; ++it)
	{
		data += *it;
		data <<= 8;
	}

	switch (last - first)
	{
	case 1:
		return static_cast<int8_t>(data);
		break;
	case 2:
		return static_cast<int16_t>(data);
		break;
	default:
		return data;
	}
}

template <typename IntegerType>
std::vector<uint8_t> GPS::int_to_raw(const IntegerType i)
{
	std::vector<uint8_t> result(sizeof(IntegerType));
	const uint8_t* byte = reinterpret_cast<const uint8_t*>(&i);
	for (uint_t i=0; i<sizeof(IntegerType); i++)
	{
		result[i] = byte[i];
	}
	return result;
}

#endif
