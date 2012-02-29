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

#ifndef IMU_H_
#define IMU_H_

/* Boost Headers */
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/smart_ptr/scoped_ptr.hpp>

/* STL Headers */
#include <vector>
#include <queue>
#include <map>

/* Project Headers */
#include "IMU_Filter.h"

/**
 * @brief This class contains all the code for interacting with the 3DM-GX3 IMU.
 *
 * The data received from the imu is stored as data members of this class and can be accessed
 * using the public threadsafe accessors.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 2, 2012
 */
class IMU
{
public:
	static IMU* getInstance();
	virtual ~IMU();

	class read_serial;
	class send_serial;
	class message_parser;
	class ack_handler;

	enum FIELD_DESCRIPTOR
	{
		COMMAND_BASE = 0x01,
		COMMAND_3DM = 0x0C,
		COMMAND_NAV_FILT = 0x0D,
		COMMAND_SYS = 0x7F,
		DATA_AHRS = 0x80,
		DATA_GPS = 0x81,
		DATA_NAV = 0x82
	};

	enum GX3_MODE
	{
		STARTUP,
		INIT,
		RUNNING,
		ERROR,
		NUM_GX3_MODES
	};

	/// get position in local ned frame (origin must be set)
	blas::vector<double> get_ned_position() const;
	/// get llh position
	inline blas::vector<double> get_llh_position() const {return get_position();}
	/// get ecef position
	inline blas::vector<double> get_ecef_position() const {return llh2ecef(get_position());}
	/// get llh ned origin
	inline blas::vector<double> get_llh_origin() const {return get_ned_origin();}
	/// get ecef ned origin
	inline blas::vector<double> get_ecef_origin() const {return llh2ecef(get_ned_origin());}
	/// public interface to set the origin
	inline void set_ned_origin() {set_ned_origin(get_llh_position());}
	/// function to keep names symmetrical with position
	inline blas::vector<double> get_ned_velocity() const {return get_velocity();}
	/// threadsafe get velocity (ned)
	inline blas::vector<double> get_velocity() const {boost::mutex::scoped_lock lock(velocity_lock); return velocity;}
	/// get rotation matrix depending on use_nav_attitude
	inline blas::matrix<double> get_rotation() const {return euler_to_rotation(get_euler());}
	/// threadsafe get rotation
	inline blas::matrix<double> get_nav_rotation() const {boost::mutex::scoped_lock lock(nav_rotation_lock); return nav_rotation;}
	/// get the euler angles depending on use_nav_attitude
	inline blas::vector<double> get_euler() const {return (get_use_nav_attitude()?get_nav_euler():get_ahrs_euler());}
	/// threadsafe get nav_euler
	inline blas::vector<double> get_nav_euler() const {boost::mutex::scoped_lock lock(nav_euler_lock); return nav_euler;}
	/// threadsafe get ahrs_euler
	inline blas::vector<double> get_ahrs_euler() const {boost::mutex::scoped_lock lock(ahrs_euler_lock); return ahrs_euler;}
	/// get nav or ahrs angular rate according to use_nav_attitude
	inline blas::vector<double> get_angular_rate() const {return (get_use_nav_attitude()?get_nav_angular_rate():get_ahrs_angular_rate());}
	/// threadsafe get nav angular_rate
	inline blas::vector<double> get_nav_angular_rate() const {boost::mutex::scoped_lock lock(nav_angular_rate_lock); return nav_angular_rate;}
	/// threadsafe get ahrs angular_rate
	inline blas::vector<double> get_ahrs_angular_rate() const {boost::mutex::scoped_lock lock(ahrs_angular_rate_lock); return ahrs_angular_rate;}
	/// get the euler angle derivatives
	blas::vector<double> get_euler_rate() const;

	/// signal to notify when gx3 mode changes
	boost::signals2::signal<void (GX3_MODE)> gx3_mode_changed;
	/// signal with status message
	boost::signals2::signal<void (std::string)> gx3_status_message;

	static blas::matrix<double> euler_to_rotation(const blas::vector<double>& euler);

private:
	IMU();
	static IMU* _instance;
	static boost::mutex _instance_lock;

	/// thread to send data on serial port
//	boost::thread send_thread;
	boost::scoped_ptr<send_serial> _send_serial;
	/// thread to receive data on serial port
	boost::thread receive_thread;
	/// thread to parse messages received from imu
	boost::thread parse_thread;

	/// serial port file descriptor
	int fd_ser;
	/// path to serial device connected to gx3
	static const std::string serial_port;
	/// initialize the serial port
	void init_serial();
	/// close the serial port
	void close_serial();

	/// compute the checksum for the imu data packet
	static std::vector<uint8_t> compute_checksum(std::vector<uint8_t> data);

	/// container for command messages
	std::queue<std::vector<uint8_t> > command_queue;
	/// serialize access to IMU::command_queue
	mutable boost::mutex command_queue_lock;
	/// threadsafe command_queue pop
	template<typename T> T command_queue_pop();
	/// threadsafe command_queue push
	template<typename T> void command_queue_push(const T& message);
	/// threadsafe empty for command_queue
	inline bool command_queue_empty() const {boost::mutex::scoped_lock lock(command_queue_lock); return command_queue.empty();}

	/// container for ahrs data
	std::queue<std::vector<uint8_t> > ahrs_queue;
	/// serialize access to IMU::ahrs_queue
	mutable boost::mutex ahrs_queue_lock;
	/// threadsafe ahrs_queue pop
	template<typename T> T ahrs_queue_pop();
	/// threadsafe ahrs_queue push
	template<typename T> void ahrs_queue_push(const T& message);
	/// threadsafe empty for ahrs_queue
	inline bool ahrs_queue_empty() const {boost::mutex::scoped_lock lock(ahrs_queue_lock); return ahrs_queue.empty();}

	/// container for gps data
	std::queue<std::vector<uint8_t> > gps_queue;
	/// serialize access to IMU::gps_queue
	mutable boost::mutex gps_queue_lock;
	/// threadsafe pop for gps_queue
	template<typename T> T gps_queue_pop();
	/// threadsafe push for gps_queue
	template<typename T> void gps_queue_push(const T& message);
	/// threadsafe empty for gps_queue
	inline bool gps_queue_empty() const {boost::mutex::scoped_lock lock(gps_queue_lock); return gps_queue.empty();}

	/// container for nav filter messages
	std::queue<std::vector<uint8_t> > nav_queue;
	/// serialize access to IMU::nav_queue
	mutable boost::mutex nav_queue_lock;
	/// threadsafe pop for nav_queue
	template<typename T> T nav_queue_pop();
	/// threadsafe push for nav_queue
	template<typename T> void nav_queue_push(const T& message);
	/// threadsafe empty for nav_queue
	inline bool nav_queue_empty() const {boost::mutex::scoped_lock lock(nav_queue_lock); return nav_queue.empty();}

	/// keep track of the gx3's mode
	GX3_MODE gx3_mode;
	/// serialize access to IMU::gx3_mode
	mutable boost::mutex gx3_mode_lock;
	/// threadsafe mode set.  emits IMU::gx3_mode_changed if mode has changed
	void set_gx3_mode(GX3_MODE mode);
	/// threadsafe get mode
	inline GX3_MODE get_gx3_mode() const {boost::mutex::scoped_lock lock(gx3_mode_lock); return gx3_mode;}


	/// signal when covariance of position measurement is too high (pos measurement invalid)
	boost::signals2::signal<void ()> pos_cov_high;
	/// signal when covariance of velocity measurement is too high
	boost::signals2::signal<void ()> vel_cov_high;
	/// signal when covariance of attitude measruement is too high
	boost::signals2::signal<void ()> att_cov_high;

	/// store the current position
	blas::vector<double> position;
	/// serialize access to IMU::position
	mutable boost::mutex position_lock;
	/// threadsafe set position
	inline void set_position(const blas::vector<double>& position) {boost::mutex::scoped_lock lock(position_lock); this->position = position;}
	/// threadsafe get position
	inline blas::vector<double> get_position() const {boost::mutex::scoped_lock lock(position_lock); return position;}

	/// store the current ned origin (in llh)
	blas::vector<double> ned_origin;
	/// serialize access to ned_origin
	mutable boost::mutex ned_origin_lock;
	/// threadsafe set ned origin @arg origin in llh
	void set_ned_origin(const blas::vector<double>& origin);
	/// threadsafe get ned origin (in llh)
	inline blas::vector<double> get_ned_origin() const {boost::mutex::scoped_lock lock(ned_origin_lock); return ned_origin;}

	/// store current velocity in ned coords
	blas::vector<double> velocity;
	/// serialize access to IMU::velocity
	mutable boost::mutex velocity_lock;
	/// threadsafe set velocity
	inline void set_velocity(const blas::vector<double>& velocity) {boost::mutex::scoped_lock lock(velocity_lock); this->velocity = velocity;}

	/// use nav attitude measurements if true, ahrs if false
	bool use_nav_attitude;
	/// serialize access to use_nav_attitude
	mutable boost::mutex use_nav_attitude_lock;
	/// threadsafe get use_nav_attitude
	inline bool get_use_nav_attitude() const {boost::mutex::scoped_lock lock(use_nav_attitude_lock); return use_nav_attitude;}
	/// threadsafe set use_nav_attitude
	void set_use_nav_attitude(bool attitude_source);
	/// connection to allow use_nav_attitude to be set from qgc
	boost::signals2::scoped_connection attitude_source_connection;

	/// store the current euler angle estimate from the nav filter
	blas::vector<double> nav_euler;
	/// serialize access to nav_euler
	mutable boost::mutex nav_euler_lock;
	/// threadsafe set nav_euler
	inline void set_nav_euler(const blas::vector<double>& euler) {boost::mutex::scoped_lock lock(nav_euler_lock); nav_euler = euler;}

	/// store the current euler angle estimate from the ahrs filter
	blas::vector<double> ahrs_euler;
	/// serialize access to ahrs_euler
	mutable boost::mutex ahrs_euler_lock;
	/// threadsafe set ahrs_euler
	inline void set_ahrs_euler(const blas::vector<double>& euler) {boost::mutex::scoped_lock lock(ahrs_euler_lock); ahrs_euler = euler;}

	/// store the current rotation matrix between ned and body frames
	blas::matrix<double> nav_rotation;
	/// serialize access to IMU::rotation
	mutable boost::mutex nav_rotation_lock;
	/// threadsafe set rotation
	inline void set_nav_rotation(const blas::matrix<double>& rotation) {boost::mutex::scoped_lock lock(nav_rotation_lock); this->nav_rotation = rotation;}

	/// store the current angular rates
	blas::vector<double> nav_angular_rate;
	/// serialize access to IMU::angular_rate
	mutable boost::mutex nav_angular_rate_lock;
	/// threadsafe set angular_rate
	inline void set_nav_angular_rate(const blas::vector<double>& angular_rate) {boost::mutex::scoped_lock lock(nav_angular_rate_lock); nav_angular_rate = angular_rate;}


	/// store the current angular rates
	blas::vector<double> ahrs_angular_rate;
	/// serialize access to IMU::angular_rate
	mutable boost::mutex ahrs_angular_rate_lock;
	/// threadsafe set angular_rate
	inline void set_ahrs_angular_rate(const blas::vector<double>& angular_rate) {boost::mutex::scoped_lock lock(ahrs_angular_rate_lock); ahrs_angular_rate = angular_rate;}


	/// send ack/nack signal when received from imu
	boost::signals2::signal<void (std::vector<uint8_t>)> ack;

	/// convert llh to ecef
	static blas::vector<double> llh2ecef(const blas::vector<double>& llh_deg);
};

template<typename T> T IMU::command_queue_pop()
{
	boost::mutex::scoped_lock lock(command_queue_lock);
	T message = command_queue.front();
	command_queue.pop();
	return message;
}

template<typename T> void IMU::command_queue_push(const T& message)
{
	boost::mutex::scoped_lock lock(command_queue_lock);
	command_queue.push(message);
}



template<typename T> void IMU::ahrs_queue_push(const T& message)
{
	boost::mutex::scoped_lock lock(ahrs_queue_lock);
	ahrs_queue.push(message);
}

template<typename T> T IMU::ahrs_queue_pop()
{
	boost::mutex::scoped_lock lock(ahrs_queue_lock);
	T message = ahrs_queue.front();
	ahrs_queue.pop();
	return message;
}

template<typename T> void IMU::gps_queue_push(const T& message)
{
	boost::mutex::scoped_lock lock(gps_queue_lock);
	gps_queue.push(message);
}

template<typename T> T IMU::gps_queue_pop()
{
	boost::mutex::scoped_lock lock(gps_queue_lock);
	T message = gps_queue.front();
	gps_queue.pop();
	return message;
}

template<typename T> void IMU::nav_queue_push(const T& message)
{
	boost::mutex::scoped_lock lock(nav_queue_lock);
	nav_queue.push(message);
}
template<typename T> T IMU::nav_queue_pop()
{
	boost::mutex::scoped_lock lock(nav_queue_lock);
	T message = nav_queue.front();
	nav_queue.pop();
	return message;
}


#endif /* IMU_H_ */
