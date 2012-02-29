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

#ifndef QGCLINK_H_
#define QGCLINK_H_

/* Project Headers */
#include "Parameter.h"

/* Boost Headers */
#include <boost/asio.hpp>
using boost::asio::ip::udp;
using boost::asio::ip::address;
#include <boost/thread.hpp>
#include <boost/signals2/signal.hpp>


/* STL Headers */
#include <vector>
#include <queue>

/**
 *  @brief Sends and receives data to QGroundControl via UDP.
 *  This class handles all Mavlink communication to QGC.  It spawns a receive thread
 *  to receive data from a UDP socket, and a send thread to send data on the same UDP
 *  socket.
 *  @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 *  @date July 8, 2011 : Created class
 *  @date October 20, 2011 : Modified send/receive calls to catch system_failure exception - ethernet cable can now be safely unplugged
 *  @date February 6, 2012: Broke class into multiple files
  */

class QGCLink
{

public:
	/// returns the running instance of QGCLink
	static QGCLink* getInstance();

	/// emitted when a shutdown message is received
	boost::signals2::signal<void ()> shutdown;
	/// emitted when a filter reset message is received
	boost::signals2::signal<void ()> reset_filter;
	/// emitted with a filter init message is received
	boost::signals2::signal<void ()> init_filter;
	/// emitted when a set origin message is received
	boost::signals2::signal<void ()> set_origin;
	/** emitted when a message changes the attitude
	 * true means use nav attitude, false means use ahrs
	 */
	boost::signals2::signal<void (bool)> attitude_source;
	/// emitted when qgc requests a servo source change
	boost::signals2::signal<void (heli::AUTOPILOT_MODE)> servo_source;
	/// emitted when qgc requests a control mode change
	boost::signals2::signal<void (heli::Controller_Mode)> control_mode;

private:

	class QGCReceive;
	class QGCSend;

	/// Construct QGCLink class
	QGCLink();
	/// Pointer to instance of QGCLink
	static QGCLink* _instance;
	/// Mutex to make class instantiation threadsafe
	static boost::mutex _instance_lock;
	/// creates the udp socket to qgc and spawns the receive and send threads
	void init();

	/// mutex to make access to send_queue threadsafe
//	boost::mutex send_queue_lock;

	udp::endpoint qgc;
	boost::asio::io_service io_service;
	udp::socket socket;

	/// thread to receive data from qgc see QGCLink::QGCReceive
	boost::thread receive_thread;
	/// thread to send data to qgc see QGCLink::QGCSend
	boost::thread send_thread;

	/// frequency to send heartbeat messages.  also used for system status messages
	int heartbeat_rate;
	/// serialized access to heartbeat_rate
	mutable boost::mutex heartbeat_rate_lock;
	/// threadsafe get heartbeat_rate
	inline int get_heartbeat_rate() const {boost::mutex::scoped_lock lock(heartbeat_rate_lock); return heartbeat_rate;}
	///threadsafe set heartbeat_rate
	inline void set_heartbeat_reate(int rate) {boost::mutex::scoped_lock lock(heartbeat_rate_lock); heartbeat_rate = rate;}

	/// frequency to send rc channel measurements
	int rc_channel_rate;
	/// serialize access to rc_channel_rate
	mutable boost::mutex rc_channel_rate_lock;
	/// threadsafe get rc_channel_rate
	inline int get_rc_channel_rate() const {boost::mutex::scoped_lock lock(rc_channel_rate_lock); return rc_channel_rate;}
	/// threadsafe set rc_channel_rate
	inline void set_rc_channel_rate(int rate) {boost::mutex::scoped_lock lock(rc_channel_rate_lock); rc_channel_rate = rate;}

	/// frequency to send control output
	int control_output_rate;
	/// serialize access to control_output_rate
	mutable boost::mutex control_output_rate_lock;
	/// threadsafe get control_output_rate
	inline int get_control_output_rate() const {boost::mutex::scoped_lock lock(control_output_rate_lock); return control_output_rate;}
	/// threadsafe set control output rate
	inline void set_control_output_rate(int rate) {boost::mutex::scoped_lock lock(control_output_rate_lock); control_output_rate = rate;}

	/// store frequency to send position measurement rate
	int position_rate;
	/// serialize access to position_rate
	mutable boost::mutex position_rate_lock;
	/// threadsafe get position rate
	inline int get_position_rate() const {boost::mutex::scoped_lock lock(position_rate_lock); return position_rate;}
	/// threadsafe set position rate
	inline void set_position_rate(int rate) {boost::mutex::scoped_lock lock(position_rate_lock); position_rate = rate;}

	/// Store desired transmission rate for attitude state estimate
	int attitude_rate;
	/// Mutex to protect QGCLink::ahrs_attitude_rate
	mutable boost::mutex attitude_rate_lock;
	/// treadsafe attitude rate set
	inline void set_attitude_rate(int rate) {boost::mutex::scoped_lock lock(attitude_rate_lock); attitude_rate = rate;}
	/// threadsafe get attitude rate
	inline int get_attitude_rate() const {boost::mutex::scoped_lock lock(attitude_rate_lock); return attitude_rate;}

	mutable boost::mutex param_recv_lock;
	bool param_recv;

	mutable boost::mutex requested_params_lock;
	std::queue<Parameter> requested_params;

	/// store whether rc_calibration packet has been requested
	bool requested_rc_calibration;
	/// serialize access to requested_rc_calibration
	mutable boost::mutex requested_rc_calibration_lock;
	/// threadsafe set requested rc calibration (could be used as slot)
	inline void set_requested_rc_calibration() {boost::mutex::scoped_lock lock(requested_rc_calibration_lock); requested_rc_calibration = true;}
	/// threadsafe clear requested rc calibration
	inline void clear_requested_rc_calibration() {boost::mutex::scoped_lock lock(requested_rc_calibration_lock); requested_rc_calibration = false;}
	/// threadsafe get requested rc calibration
	inline bool get_requested_rc_calibration() const {boost::mutex::scoped_lock lock(requested_rc_calibration_lock); return requested_rc_calibration;}

	int uasId;
};

#endif
