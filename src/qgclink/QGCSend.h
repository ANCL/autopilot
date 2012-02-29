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

#ifndef QGCSEND_H_
#define QGCSEND_H_

/* Project Headers */
#include "QGCLink.h"
#include "heli.h"
#include "IMU.h"

/* STL Headers */
#include <queue>
#include <vector>

/* Boost Headers */
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/**
 * Send data over UDP to QGroundControl
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 *
 */
class QGCLink::QGCSend
{
public:
	QGCSend(QGCLink* parent = 0);
	/// Threadsafe copy constructor
	QGCSend(const QGCSend& other);
	~QGCSend();

	/// threadsafe assignment operator
	QGCSend& operator=(const QGCSend& other);
	/// entry point into thread
	void operator()(){send();}

private:
	/// Pointer to QGCLink instance
	QGCLink *qgc;

	/// queue to store the message to be sent
	std::queue<std::vector<uint8_t> > *send_queue;

	/** queue stream messages and perform actual send */
	void send();

	/** queue up a heartbeat message
	 * @param sendq queue to put heartbeat message in */
	void send_heartbeat(std::queue<std::vector<uint8_t> >* sendq);
	/** queue up a raw imu sense message
	 * @param sendq queue to put message in
	 */
	void send_raw_imu(std::queue<std::vector<uint8_t> >* sendq);
	/** queue up rc channel (raw and scaled) message
	 * @param sendq queue to put messages in
	 */
	void send_rc_channels(std::queue<std::vector<uint8_t> >* sendq);
	/** queue up status message
	 * @param sendq queue to put messages in
	 */
	void send_status(std::queue<std::vector<uint8_t> >* sendq);
	/** queue up attitude estimate message
	 * @param sendq queue to put messages in
	 */
	void send_attitude(std::queue<std::vector<uint8_t> > *sendq);
	/** queue up control effort stream message
	 * @param sendq queue to add message to
	 */
	void send_control_effort(std::queue<std::vector<uint8_t> > *sendq);
	/// send the current position and velocity measurement
	void send_position(std::queue<std::vector<uint8_t> > *sendq);
	/** queue up parameter list message
	 * @param sendq queue to put messages in
	 */
	void send_param(std::queue<std::vector<uint8_t> > *sendq);
	/** queue up requested parameter message
	 * @param sendq queue to put messages in
	 */
	void send_requested_params(std::queue<std::vector<uint8_t> > *sendq);
	/** add rc calibration message to send queue
	 * @param sendq send queue
	 */
	void send_rc_calibration(std::queue<std::vector<uint8_t> > *sendq);
	void send_console_message(const std::string& message, std::queue<std::vector<uint8_t> > *sendq);
	/** determine whether to send a particular stream
	 * @param stream_rate requested stream rate
	 * @param send_rate rate at which the send loop runs
	 * @param count number of itereates of the send loop
	 * @returns true if stream should be send at this iteration, false if not.
	 */
	bool should_run(int stream_rate, int send_rate, int count);

	/// store the current servo source
	heli::AUTOPILOT_MODE servo_source;
	/// serialize access to servo_source
	mutable boost::mutex servo_source_lock;
	/// threadsafe get servo source
	inline heli::AUTOPILOT_MODE get_servo_source() const {boost::mutex::scoped_lock lock(servo_source_lock); return servo_source;}
	/// threadsafe set servo source
	inline void set_servo_source(heli::AUTOPILOT_MODE servo_source) {boost::mutex::scoped_lock lock(servo_source_lock); this->servo_source = servo_source;}

	/// store the current pilot mode
	heli::PILOT_MODE pilot_mode;
	/// serialize access to pilot_mode
	mutable boost::mutex pilot_mode_lock;
	/// thread safe get pilot_mode
	inline heli::PILOT_MODE get_pilot_mode() const {boost::mutex::scoped_lock lock(pilot_mode_lock); return pilot_mode;}
	/// thread safe set pilot_mode
	inline void set_pilot_mode(heli::PILOT_MODE pilot_mode) {boost::mutex::scoped_lock lock(pilot_mode_lock); this->pilot_mode = pilot_mode;}

	/// store the current gx3 filter state
	IMU::GX3_MODE filter_state;
	/// serialize access to filter_state;
	mutable boost::mutex filter_state_lock;
	/// threadsafe get_filter_state
	inline IMU::GX3_MODE get_filter_state() const {boost::mutex::scoped_lock lock(filter_state_lock); return filter_state;}
	/// threadsafe set filter_state
	inline void set_filter_state(IMU::GX3_MODE filter_state) {boost::mutex::scoped_lock lock(filter_state_lock); this->filter_state = filter_state;}

	/// store the current controller mode
	heli::Controller_Mode control_mode;
	/// serialize access to contorl_mode
	mutable boost::mutex control_mode_lock;
	/// threadsafe get control_mode
	inline heli::Controller_Mode get_control_mode() const {boost::mutex::scoped_lock lock(control_mode_lock); return control_mode;}
	/// threadsafe set control_mode
	inline void set_control_mode(heli::Controller_Mode control_mode) {boost::mutex::scoped_lock lock(control_mode_lock); this->control_mode = control_mode;}

	/// true if there is a new message from the gx3
	bool gx3_new_message;
	/// serialize access to gx3_new_message
	mutable boost::mutex gx3_new_message_lock;
	/// threadsafe clear gx3_new_message
	inline void clear_gx3_new_message() {boost::mutex::scoped_lock lock(gx3_new_message_lock); gx3_new_message = false;}
	/// threadsafe set gx3_new_message
	inline void set_gx3_new_message() {boost::mutex::scoped_lock lock(gx3_new_message_lock); gx3_new_message = true;}
	/// threadsafe get gx3_new_message
	inline bool get_gx3_new_message() const {boost::mutex::scoped_lock lock(gx3_new_message_lock); return gx3_new_message;}

	/// string to store message from gx3
	std::string gx3_message;
	/// serialize access to gx3_message
	mutable boost::mutex gx3_message_lock;
	/// threadsafe access gx3_message
	inline std::string get_gx3_message() const {boost::mutex::scoped_lock lock(gx3_message_lock); return gx3_message;}
	/// threadsafe set gx3_messsage (can be slot)
	inline void set_gx3_message(std::string message) {boost::mutex::scoped_lock lock(gx3_message_lock); gx3_message = message; set_gx3_new_message();}
	/// function to send the gx3_message to qgc
	void send_gx3_message(std::queue<std::vector<uint8_t> > *sendq);

	/// store the current attitude source
	bool attitude_source;
	/// serialize access to attitude_source
	mutable boost::mutex attitude_source_lock;
	/// threadsafe get attitude source
	inline bool get_attitude_source() const {boost::mutex::scoped_lock lock(attitude_source_lock); return attitude_source;}
	/// threadsafe set attitude source
	inline void set_attitude_source(bool source) {boost::mutex::scoped_lock lock(attitude_source_lock); attitude_source = source;}
	/// connection for attitude source
	boost::signals2::scoped_connection attitude_source_connection;

	/// store a queue of messages
	std::queue<std::string> message_queue;
	/// serialize access to message_queue
	mutable boost::mutex message_queue_lock;
	/// threadsafe push message
	inline void message_queue_push(const std::string& message) {boost::mutex::scoped_lock lock(message_queue_lock); message_queue.push(message);}
	/// threadsafe empty test
	inline bool message_queue_empty() const {boost::mutex::scoped_lock lock(message_queue_lock); return message_queue.empty();}
	/// threadsafe pop message
	std::string message_queue_pop();

	/// Stores the time when the class is instantiated (i.e., the program starts)
	boost::posix_time::ptime startTime;
};
#endif
