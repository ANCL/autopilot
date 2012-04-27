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

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/signals2/signal.hpp>
#include <boost/thread.hpp>

/* Project Headers */
#include "Debug.h"
#include "gps_time.h"

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
 * @date April 27, 2012: Completed work to integrate novatel with gx3
 */

class GPS
{
public:
	virtual ~GPS();

	static GPS* getInstance();

	class read_serial;

	/// threadsafe get llh_position
	inline blas::vector<double> get_llh_position() const {boost::mutex::scoped_lock(llh_position_lock); return llh_position;}
	/// threadsafe get ned_velocity
	inline blas::vector<double> get_ned_velocity() const {boost::mutex::scoped_lock(ned_velocity_lock); return ned_velocity;}
	/// threadafe get position error std dev
	inline blas::vector<double> get_pos_sigma() const {boost::mutex::scoped_lock(pos_sigma_lock); return pos_sigma;}
	/// threadsafe get vel error std dev
	inline blas::vector<double> get_vel_sigma() const {boost::mutex::scoped_lock(vel_sigma_lock); return vel_sigma;}
	/// threadsafe get gps time
	inline gps_time get_gps_time() const {boost::mutex::scoped_lock(_gps_time_lock); return _gps_time;}

	/// signal when gps measurement is updated
	boost::signals2<void ()> gps_updated;

private:

	/// default constructor
	GPS();
	/// pointer to <em> the </em> instance of GPS
	static GPS* _instance;
	/// serialize access to _instance
	static boost::mutex _instance_lock;

	/// thread used to communicate with the GPS
	boost::thread read_serial_thread;

	/// container for llh_position
	blas::vector<double> llh_position;
	/// serialize access to llh_position
	boost::mutex llh_position_lock;
	/// threadsafe set llh_position
	inline void set_llh_position(const boost::vector<double>& llh) {boost::mutex::scoped_lock(llh_position_lock); llh_position = llh;}

	/// container for ned_velocity
	blas::vector<double> ned_velocity;
	/// serialize access to ned_velocity
	boost::mutex ned_velocity_lock;
	/// threadsafe set ned_velocity
	inline void set_ned_velocity(const boost::vector<double>& ned_vel) {boost::mutex::scoped_lock(ned_velocity_lock); ned_velocity = ned_vel;}

	/// container for pos error std dev
	blas::vector<double> pos_sigma;
	/// serialize access to pos_sigma
	boost::mutex pos_sigma_lock;
	/// threadsafe set pos_sigma
	inline void set_pos_sigma(const boost::vector<double>& pos_error) {boost::mutex::scoped_lock(pos_sigma_lock); pos_sigma = pos_error;}

	/// container for velocity error std dev
	blas::vector<double> vel_sigma;
	/// serialize access to vel_sigma
	boost::mutex vel_sigma_lock;
	/// threadsafe set vel sigma
	inline void set_vel_sigma(const boost::vector<double>& vel_error) {boost::mutex::scoped_lock(vel_sigma_lock); vel_sigma = vel_error;}

	/// container for gps_time
	gps_time _gps_time;
	/// serialize access to gps_time
	boost::mutex _gps_time_lock;
	/// threadsafe set gps_time
	inline void set_gps_time(const gps_time& time) {boost::mutex::scoped_lock(_gps_time_lock); _gps_time = time;}

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
