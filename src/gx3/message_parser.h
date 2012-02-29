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

#ifndef MESSAGE_PARSER_H_
#define MESSAGE_PARSER_H_

#include "IMU.h"

/* STL HEADERS */
#include <bitset>

/**
 * Message parsing class to handle the GX3 message protocol
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 2, 2012: Class Creation
 */
class IMU::message_parser
{
public:
	message_parser();
	virtual ~message_parser();
	void operator()();

private:
	/// parse nav filter data message and take appropriate action
	void parse_nav_message(const std::vector<uint8_t>& message);
	/// parse command message and take appropriate action
	void parse_command_message(const std::vector<uint8_t>& message);
	/// parse ahrs message and take appropriate action
	void parse_ahrs_message(const std::vector<uint8_t>& message);

	/** convert raw binary representation of a double precision
	 * floating point number which is stored between first and last into
	 * a double representation.  This
	 * function assumes big endian byte storage.
	 */
	template <typename InputIterator>
	static double raw_to_double(InputIterator first, InputIterator last);
	/// same as IMU::message_parser::raw_to_double except assumes sizeof(double) number of bytes
	template <typename InputIterator>
	static double raw_to_double(InputIterator first) {return raw_to_double(first, first + sizeof(double));}
	/** convert raw binary representation of a single precision
	 * floating point number which is stored between first and last into a float
	 * representation.  This
	 * function assumes big endian byte storage.
	 */
	template <typename InputIterator>
	static float raw_to_float(InputIterator first, InputIterator last);
	/// same as IMU::message_parser::raw_to_float except assumes sizeof(float) number of bytes
	template <typename InputIterator>
	static float raw_to_float(InputIterator first) {return raw_to_float(first, first + sizeof(float));}

	/// store the status flags for the ins kalman.  does not need mutex since it isn't used outside this thread.
	std::bitset<16> nav_status_flags;

	/// filters for the nav gyro measurements
	boost::array<IMU_Filter, 3> nav_filters;

	/// filters for the ahrs gyro measurements
	boost::array<IMU_Filter, 3> ahrs_filters;

};

template <typename InputIterator>
double IMU::message_parser::raw_to_double(InputIterator first, InputIterator last)
{
	uint64_t result = 0;
	for (InputIterator it = first; it != last; ++it)
	{
		result <<= 8;
		result += *it;
	}
	return *reinterpret_cast<double*>(&result);
}

template <typename InputIterator>
float IMU::message_parser::raw_to_float(InputIterator first, InputIterator last)
{
	uint32_t result = 0;
	for (InputIterator it = first; it != last; ++it)
	{
		result <<= 8;
		result += *it;
	}
	return *reinterpret_cast<float*>(&result);
}

#endif
