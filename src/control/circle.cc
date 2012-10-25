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

#include "circle.h"

/* Boost Headers */
#include <boost/math/constants/constants.hpp>

/* STL Headers */
#include <math.h>

/* Project Headers */
#include "IMU.h"

circle::circle()
: radius(0),
  start_location(blas::zero_vector<double>(3)),
  center_location(blas::zero_vector<double>(3)),
  initial_angle(0)
{


}

void circle::reset()
{
	set_start_location(IMU::getInstance()->get_ned_position());
	set_start_time();
	set_center_location();

	blas::vector<double> initial_vector(get_start_location() - get_center_location());
	set_initial_angle(atan2(initial_vector(1), initial_vector(0)));
}

blas::vector<double> circle::get_reference_position() const
{
	double elapsed_time = (boost::posix_time::microsec_clock::local_time() - get_start_time()).total_milliseconds()/1000.0;
	double period = (get_speed() > 0 ? get_circumference()/get_speed() : 0);
	double hover_time = get_hover_time();
	if (period == 0 || elapsed_time <= hover_time)
	{
		return get_start_location();
	}
	else
	{
		elapsed_time -= hover_time;
		blas::vector<double> reference_position(get_center_location());
		static double pi = boost::math::constants::pi<double>();
		reference_position(0) += get_radius()*cos(2*pi*elapsed_time/period + get_initial_angle());
		reference_position(1) += get_radius()*sin(2*pi*elapsed_time/period + get_initial_angle());
		return reference_position;
	}
}

double circle::get_circumference() const
{
	return 2*boost::math::constants::pi<double>()*get_radius();
}

void circle::set_center_location()
{
	blas::vector<double> center(blas::zero_vector<double>(3));
	center(0) = get_radius();  // vector in body frame with origin at heli
	boost::mutex::scoped_lock lock(center_location_lock);
	center_location = prod(IMU::getInstance()->get_heading_rotation(), center) + get_start_location();
}
