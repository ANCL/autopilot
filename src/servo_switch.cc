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

#include "servo_switch.h"

servo_switch* servo_switch::_instance = NULL;
boost::mutex servo_switch::_instance_lock;

servo_switch* servo_switch::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new servo_switch;
	return _instance;
}

servo_switch::servo_switch()
: raw_inputs(9, 0),
  raw_outputs(9,0),
  pilot_mode(heli::PILOT_UNKNOWN)
{
	init_port();
	receive = boost::thread(read_serial());
	send = boost::thread(send_serial());
	LogFile *log = LogFile::getInstance();
	log->logHeader(heli::LOG_INPUT_PULSE_WIDTHS, "CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9");
	log->logHeader(heli::LOG_OUTPUT_PULSE_WIDTHS, "CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9");
	log->logHeader(heli::LOG_INPUT_RPM, "RPM");
}

servo_switch::~servo_switch()
{

}

void servo_switch::init_port()
{
	std::string port = "/dev/ser3";

	boost::mutex::scoped_lock lock(fd_ser1_lock);
	fd_ser1 = open(port.c_str(), O_RDWR);       // GPS is connected to serial port#2
	if(fd_ser1 == -1)
	{
		critical() << "Unable to open port " << port;
	}

	struct termios port_config;

	tcgetattr(fd_ser1, &port_config);                  // get the current port settings
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
	tcflow (fd_ser1, TCION);                           // set -ispaged
	tcflow (fd_ser1, TCOON);                           // set -ospaged
	tcflow (fd_ser1, TCIONHW);                         // set -ihpaged
	tcflow (fd_ser1, TCOONHW);                         // set -ohpaged

	if (cfsetospeed(&port_config, B115200) != 0)
		critical() << "could not set output speed";
	if (cfsetispeed(&port_config, B115200) != 0)
		critical() << "could not set input speed";
	if (tcsetattr(fd_ser1, TCSADRAIN, &port_config) != 0)
		critical() << "could not set serial port attributes";
	tcgetattr(fd_ser1, &port_config);
	tcflush(fd_ser1, TCIOFLUSH);
}

void servo_switch::set_pilot_mode(heli::PILOT_MODE mode)
{
	boost::mutex::scoped_lock lock(pilot_mode_lock);
	if (pilot_mode == mode)
		return;
	pilot_mode = mode;
	message() << "Pilot mode changed to: " << pilot_mode_string(mode);
	pilot_mode_changed(mode);
}

std::string servo_switch::pilot_mode_string(heli::PILOT_MODE mode)
{
	switch(mode)
	{
	case heli::PILOT_MANUAL:
		return "manual";
	case heli::PILOT_AUTO:
		return "auto";
	default:
		return "unknown mode";
	}
}

std::vector<uint8_t> servo_switch::compute_checksum(uint8_t id, uint8_t count, const std::vector<uint8_t>& payload)
{
	std::vector<uint8_t> checksum(2, 0);
	checksum[0] = id + count;
	checksum[1] = 2*id + count;

	for (std::vector<uint8_t>::const_iterator it = payload.begin(); it != payload.end(); ++it)
	{
		checksum[0] += *it;
		checksum[1] += checksum[0];
	}
	return checksum;
}

/* read_serial functions */

void servo_switch::read_serial::read_data()
{
	int fd_ser = servo_switch::getInstance()->get_serial_descriptor();
	std::vector<uint8_t> payload;
	std::vector<uint8_t> checksum(2);
	for(;;)
	{
//		debug() << "searching for header";
		find_next_header();

		// get message id
		uint8_t id;
		readcond(fd_ser, &id, 1, 1, 10, 10);

		// get message count
		uint8_t count;
		readcond(fd_ser, &count, 1, 1, 10, 10);

		// allocate space for message
		payload.resize(count);
		// get message payload
		readcond(fd_ser, &payload[0], count, count, 10, 10);

		// zero checksum
		checksum.assign(checksum.size(), 0);
		// get checksum
		readcond(fd_ser, &checksum[0], 2, 2, 10, 10);

		if (checksum == compute_checksum(id, count, payload))
			parse_message(id, payload);
	}
}

void servo_switch::read_serial::parse_message(uint8_t id, const std::vector<uint8_t>& payload)
{
	switch (id)
	{
	case 10:
	{
//		debug() << "Received status message from servo switch";
		uint16_t status =  payload[1];
		// shift right to get command channel state
		status = (status & 0x6) >> 1;
		switch (status)
		{
		case 1:
			getInstance()->set_pilot_mode(heli::PILOT_MANUAL);
			break;
		case 2:
		case 3:
			getInstance()->set_pilot_mode(heli::PILOT_AUTO);
			break;
		default:
			warning() << "Command channel state signal not present on servo switch";
		}
		break;
	}
	case 13:
	{
//		debug() << "Received pulse input message from servo switch";
		parse_pulse_inputs(payload);
		break;
	}
	case 14:
	{
		parse_aux_inputs(payload);
		break;
	}
	default:
		debug() << "Received unknown message from servo switch";
	}
}

void servo_switch::read_serial::parse_pulse_inputs(const std::vector<uint8_t>& payload)
{
	std::vector<uint16_t> pulse_inputs(9, 0);  // no need for more than 9 channels
	for (uint_t i=1; i<payload.size()/2 && i < pulse_inputs.size(); i++)
	{
		pulse_inputs[i-1] = (static_cast<uint16_t>(payload[i*2]) << 8) + payload[i*2+1];
	}
	// treat ch8 differently
	pulse_inputs[7] = (static_cast<uint16_t>(payload[0]) << 8) + payload[1];

	getInstance()->set_raw_inputs(pulse_inputs);
    LogFile *log = LogFile::getInstance();
    log->logData(heli::LOG_INPUT_PULSE_WIDTHS, pulse_inputs);
}

void servo_switch::read_serial::parse_aux_inputs(const std::vector<uint8_t>& payload)
{
	std::bitset<8> meas_byte (payload[2]);
	if(meas_byte.test(7))
	{
		debug() << "Time measurement over range";
		return ;
	}
	if(!meas_byte.test(6))
	{
		debug() << "Time measurement is not running";
	}

	meas_byte.set(7,0);
	meas_byte.set(6,0);

	uint16_t time_measurement;
	time_measurement = (static_cast<uint16_t>(meas_byte.to_ulong()) << 8) + payload[3];

	double speed = 1 / (time_measurement*32.0*0.000001);
	std::vector<double> speeds;
	speeds.push_back(speed);
	servo_switch& ss = *servo_switch::getInstance();
	if (speed < 15000.0/60.0)
	{
		ss.set_engine_speed(speed);
	}
	speeds.push_back(ss.get_engine_speed());

	LogFile *log = LogFile::getInstance();
	log->logData(heli::LOG_INPUT_RPM, speeds);
}


void servo_switch::read_serial::find_next_header()
{
	bool synchronized = false;
	int fd_ser = servo_switch::getInstance()->get_serial_descriptor();

	bool found_first_byte = false;
	uint8_t buf;
	while (!synchronized)
	{
		readcond(fd_ser, &buf, 1, 1, 10, 10);
//		debug() << "read byte: " << std::hex << buf;
		if (buf == 0x81) // first byte of header
			found_first_byte = true;
		else if (buf == 0xA1 && found_first_byte)
			synchronized = true;
		else
			found_first_byte = false;
	}
//	debug() << "found header";
}
/* send_serial functions */

void servo_switch::send_serial::send_data()
{
	struct sigevent         event;
	struct itimerspec       itime;
	timer_t                 timer_id;
	/* ChannelCreate() func. creates a channel that is owned by the process (and isn't bound to the creating thread). */
	int chid = ChannelCreate(0);

	event.sigev_notify = SIGEV_PULSE;

	/* Threads wishing to connect to the channel identified by 'chid'(channel id) by ConnectAttach() func.
		 Once attached thread can MsgSendv() & MsgSendPulse() to enqueue messages & pulses on the channel in priority order. */
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0,
			chid,
			_NTO_SIDE_CHANNEL, 0);
	event.sigev_priority = heli::servo_switch_send_priority;
	event.sigev_code = heli::servo_switch_pulse_code;
	timer_create(CLOCK_REALTIME, &event, &timer_id);

	itime.it_value.tv_sec = 0;
	itime.it_value.tv_nsec = 20000000;
	itime.it_interval.tv_sec = 0;
	itime.it_interval.tv_nsec = 20000000; // 50Hz
	timer_settime(timer_id, 0, &itime, NULL);


	_pulse pulse;

	for (;;)
	{
		MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL);
		std::vector<uint8_t> pulse_message = get_pulse_message();
		LogFile::getInstance()->logData(heli::LOG_OUTPUT_PULSE_WIDTHS, getInstance()->get_raw_outputs()); // log here to ensure raw outputs are only logged once every time data is sent
		while (write(getInstance()->get_serial_descriptor(), &pulse_message[0], pulse_message.size()) < 0)
			debug() << "Error sending pulse output message to servo switch";
	}
}

std::vector<uint8_t> servo_switch::send_serial::get_pulse_message()
{
	std::vector<uint16_t> raw_outputs(getInstance()->get_raw_outputs());

	std::vector<uint8_t> message;

	message.push_back(0x81);
	message.push_back(0xA1);
	message.push_back(20);
	message.push_back(raw_outputs.size()*2);

	// put ch8 on pin1
//	message.push_back(static_cast<uint8_t>(raw_outputs[7]>>8));
//	message.push_back(static_cast<uint8_t>(raw_outputs[7] & 0xFF));

	for (uint_t i=0; i<raw_outputs.size(); i++)
	{
//		if (i != 7)
//		{
//			if (i==2 || i==3)
//			{
//				message.push_back(static_cast<uint8_t>(raw_outputs[i-2]>>8));
//							message.push_back(static_cast<uint8_t>(raw_outputs[i-2] & 0xFF));
//			}
//			else
//			{
			message.push_back(static_cast<uint8_t>(raw_outputs[i]>>8));
			message.push_back(static_cast<uint8_t>(raw_outputs[i] & 0xFF));
//			}
//		}
	}

	std::vector<uint8_t> checksum = compute_checksum(20, raw_outputs.size()*2, std::vector<uint8_t>(message.begin()+4, message.end()));

	message.push_back(checksum[0]);
	message.push_back(checksum[1]);

	return message;
}
