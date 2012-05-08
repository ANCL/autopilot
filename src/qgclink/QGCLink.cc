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

#include "QGCLink.h"

/* Project Headers */
#include "QGCReceive.h"
#include "QGCSend.h"
#include "servo_switch.h"
#include "RCTrans.h"
#include "Control.h"

/* Boost Headers */
#include <boost/asio.hpp>
using boost::asio::ip::udp;
using boost::asio::ip::address;
#include <boost/array.hpp>

/* Mavlink Headers */
#include <mavlink.h>

/* STL Headers */
#include <vector>

QGCLink::QGCLink()
: socket(io_service),
  heartbeat_rate(10),
  rc_channel_rate(10),
  control_output_rate(10),
  position_rate(10),
  attitude_rate(10),
  requested_rc_calibration(false),
  uasId(100)
{
	init();
	param_recv = false;
}


void QGCLink::init()
{
	try
	{
#if defined(IP_ADDRESS)

		qgc.address(address::from_string(IP_ADDRESS));
		debug() << "QGCLink: Opening socket to " << qgc.address().to_string();
		qgc.port(14550);

		socket.open(udp::v4());

		receive_thread = boost::thread(QGCReceive());

		send_thread = boost::thread(QGCSend(this));
#else
#error "IP_ADDRESS not defined.  Please copy platform_settings.mk.dist to platform_settings.mk and set your IP."
#endif
	}
	catch (std::exception& e)
	{
		critical() << "QGCLink: " << e.what();
		throw e;
	}
}

QGCLink* QGCLink::_instance = NULL;
boost::mutex QGCLink::_instance_lock;

QGCLink* QGCLink::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if(!_instance)
		_instance = new QGCLink;
	return _instance;
}
