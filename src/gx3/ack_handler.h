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

#ifndef ACK_HANDLER_H_
#define ACK_HANDLER_H_

/* STL  Headers */
#include <vector>

/* Project Headers */
#include "IMU.h"

/* Boost Headers */
#include <boost/signals2/signal.hpp>

/**
 * @brief slot for handling ack/nack packets received from 3DM-GX3
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 3, 2012
 *
 * Here is some example code taken from IMU::send_serial::ping()
 * @code
 * 	std::vector<uint8_t> ping;
	ping += 0x75, 0x65, 0x01, 0x02, 0x02, 0x01;
	std::vector<uint8_t> checksum = compute_checksum(ping);
	ping.insert(ping.end(), checksum.begin(), checksum.end());

	bool ack_received = false;
	while (!ack_received)
	{
		write(IMU::getInstance()->fd_ser, &ping[0], ping.size());
		ack_handler ping_ack(0x01);
		boost::signals2::scoped_connection c(IMU::getInstance()->ack.connect(boost::bind(&ack_handler::operator(), boost::ref(ping_ack), _1)));
		ping_ack.wait_for_ack();
		if (ping_ack.get_error_code() == 0x00)
			ack_received = true;
	}
 * @endcode
 */
class IMU::ack_handler
{
public:
	/// construct ack_handler to wait for a particular ack/nack
	explicit ack_handler(uint8_t command = 0);
	/// copy constructor to handle noncopyable mutex
	ack_handler(const ack_handler& other);
	/// assignment operator to handle noncopyable mutex
	const ack_handler& operator=(const ack_handler& other);
	/// used as slot to wait for ack
	void operator()(std::vector<uint8_t> message);
	/// timeout in ms
	void wait_for_ack(int timeout = 0);
	/// extract error code byte from ack/nack message - not valid until ack received (returns 255 if ack nor received)
	uint8_t get_error_code();
private:
	class spin
	{
	public:
		explicit spin(ack_handler* parent);
		void operator()();
	private:
		ack_handler* parent;
	};
	/// flag to tell when ack is received
	bool ack_received;
	/// serialize access to IMU::ack_handler::ack_received
	mutable boost::mutex ack_received_lock;
	/// threadsafe set ack_received
	void set_ack_received() {boost::mutex::scoped_lock lock(ack_received_lock); ack_received = true;}
	/// threadsafe get ack_received
	bool get_ack_received() {boost::mutex::scoped_lock lock(ack_received_lock); return ack_received;}
	/// store the command code to wait for
	uint8_t command;
	/// store the ack/nack message
	std::vector<uint8_t> message;

	boost::signals2::scoped_connection ack_connection;
};
#endif
