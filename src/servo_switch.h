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

#ifndef SERVO_SWITCH_H_
#define SERVO_SWITCH_H_

/* Boost Headers */
#include <boost/thread.hpp>
#include <boost/signals2.hpp>

/* STL Headers */
#include <vector>

/* Project Headers */
#include "heli.h"
#include "Debug.h"

/* c headers */
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstdlib>
#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <sys/neutrino.h>
#include <bitset>

/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/**
 * @brief Contains hardware specific code for Microbotics Servo Switch.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 *
 * This class runs two threads.  One to send data on the serial port (/dev/ser3) and one to receive data.
 * The sending thread runs at 50 Hz and is used to transmit the new pulse widths commanded by the control.
 * The receive thread is used to get the current pilot inputs and the status message which indicates the state of the
 * control channel (pilot manual or pilot auto).
 * @date February 2012: Class creation
 * @date May 1, 2012: Added auxiliary input for engine speed
 */
class servo_switch
{
public:
	static servo_switch* getInstance();

	virtual ~servo_switch();

	class read_serial
	{
	public:
		void operator()() {read_data();}

	private:

		void read_data();
		void sync();
		void parse_message(uint8_t id, const std::vector<uint8_t>& payload);
		void parse_pulse_inputs(const std::vector<uint8_t>& payload);
		void parse_aux_inputs(const std::vector<uint8_t>& payload);
		void find_next_header();
	};

	class send_serial
	{
	public:
		void operator()() {send_data();}
	private:

		void send_data();
		std::vector<uint8_t> get_pulse_message();
	};

	/** get the current pilot inputs
	 */
	std::vector<uint16_t> getRaw() {return get_raw_inputs();}
	uint16_t getRaw(heli::Channel ch) {return get_raw_inputs()[ch];}
	/// set the value of the servo outputs
	void setRaw(const std::vector<uint16_t>& raw_outputs) {set_raw_outputs(raw_outputs);}
	inline void setRaw(heli::Channel ch, uint16_t pulse_width) {boost::mutex::scoped_lock lock(raw_outputs_lock); raw_outputs[ch] = pulse_width;}

	/// signal with new mode as argument
	boost::signals2::signal<void (heli::PILOT_MODE)> pilot_mode_changed;
	inline heli::PILOT_MODE get_pilot_mode() {boost::mutex::scoped_lock lock(pilot_mode_lock); return pilot_mode;}

	inline double get_engine_speed() const {boost::mutex::scoped_lock(engine_speed_lock); return engine_speed;}
	inline double get_engine_rpm() const {return get_engine_speed()*60;}
	inline double get_main_rotor_speed() const {return get_engine_speed()*90.0/13.0;}
	inline double get_main_rotor_rpm() const {return get_engine_rpm()*90.0/13.0;}

private:
	servo_switch();
	static servo_switch* _instance;
	static boost::mutex _instance_lock;

	void init_port();

	/// @returns the file descriptor of the serial port (threadsafe)
	inline int get_serial_descriptor() {boost::mutex::scoped_lock lock(fd_ser1_lock); return fd_ser1;}
	int fd_ser1;
	boost::mutex fd_ser1_lock;

	boost::thread receive;
	boost::thread send;

	static std::vector<uint8_t> compute_checksum(uint8_t id, uint8_t count, const std::vector<uint8_t>& payload);

	std::vector<uint16_t> raw_inputs;
	boost::mutex raw_inputs_lock;
	inline std::vector<uint16_t> get_raw_inputs() {boost::mutex::scoped_lock lock(raw_inputs_lock); return raw_inputs;}
	inline void set_raw_inputs(const std::vector<uint16_t>& raw_inputs) {boost::mutex::scoped_lock lock(raw_inputs_lock); this->raw_inputs = raw_inputs;}

	std::vector<uint16_t> raw_outputs;
	boost::mutex raw_outputs_lock;
	inline std::vector<uint16_t> get_raw_outputs() {boost::mutex::scoped_lock lock(raw_outputs_lock); return raw_outputs;}
	inline void set_raw_outputs(const std::vector<uint16_t>& raw_outputs) {boost::mutex::scoped_lock lock(raw_outputs_lock); this->raw_outputs = raw_outputs;}

	heli::PILOT_MODE pilot_mode;
	boost::mutex pilot_mode_lock;
	void set_pilot_mode(heli::PILOT_MODE mode);
	static std::string pilot_mode_string(heli::PILOT_MODE mode);

	double engine_speed;
	boost::mutex engine_speed_lock;
	inline void set_engine_speed(double speed) {boost::mutex::scoped_lock(engine_speed_lock); engine_speed = speed;}
};

#endif
