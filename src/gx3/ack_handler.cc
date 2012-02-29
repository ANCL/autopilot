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

#include "ack_handler.h"

/* Project Headers */
#include "Debug.h"
#include "IMU.h"

/* Boost Headers */
#include <boost/thread.hpp>
//#include <boost/bind.hpp>

IMU::ack_handler::ack_handler(uint8_t command)
:ack_received(false),
 command(command),
 ack_connection(IMU::getInstance()->ack.connect(boost::bind(&ack_handler::operator(), this, _1)))
{
}

IMU::ack_handler::ack_handler(const ack_handler& other)
:command(other.command)
{
	debug() <<"called ack_handler() copy";
	boost::mutex::scoped_lock lock(other.ack_received_lock);
	ack_received = other.ack_received;
}

const IMU::ack_handler& IMU::ack_handler::operator=(const ack_handler& other)
{
	debug() << "called ack_handler::operator=";
	if (this == &other)
		return *this;

	boost::mutex::scoped_lock lock1(&ack_received_lock < &other.ack_received_lock ? ack_received_lock : other.ack_received_lock);
	boost::mutex::scoped_lock lock2(&ack_received_lock > &other.ack_received_lock ? ack_received_lock : other.ack_received_lock);
	ack_received = other.ack_received;

	return *this;
}

void IMU::ack_handler::wait_for_ack(int timeout)
{
	boost::thread wait_thread(spin(this));
	if (timeout == 0)
		wait_thread.join();
	else
		wait_thread.timed_join(boost::posix_time::milliseconds(timeout));
}

void IMU::ack_handler::operator()(std::vector<uint8_t> message)
{
	if (message[0] == command)
	{
//		debug() << "got ack for command: " << static_cast<int>(command);
		this->message.clear();
		this->message.insert(this->message.begin(), message.begin(), message.end());
		set_ack_received();
	}
}

uint8_t IMU::ack_handler::get_error_code()
{
	if (get_ack_received())
		return message[1];
	else
		return 255;
}

IMU::ack_handler::spin::spin(ack_handler* parent)
:parent(parent)
{

}

void IMU::ack_handler::spin::operator()()
{
	while (!parent->get_ack_received())
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}
