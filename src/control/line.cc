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

#include "line.h"

/* STL Headers */
#include <math.h>

/* Project Headers */
#include "IMU.h"

line::line()
: start_location(blas::zero_vector<double>(3)),
  end_location(blas::zero_vector<double>(3)),
  hover_time(0)
{


}

void line::reset()
{
	set_start_location(IMU::getInstance()->get_ned_position());
	set_start_time();
	blas::vector<double> body_travel(3);
	body_travel(0) = get_x_travel();
	body_travel(1) = get_y_travel();
	set_end_location(get_start_location() + prod(trans(IMU::getInstance()->get_heading_rotation()), body_travel));
}

blas::vector<double> line::get_reference_position() const
{
	double elapsed_time = (boost::posix_time::microsec_clock::local_time() - get_start_time()).total_milliseconds()/1000.0;
	double flight_time = (get_speed() > 0 ? get_distance()/get_speed() : 0);
	double hover_time = get_hover_time();
	if (flight_time == 0 || elapsed_time <= hover_time)
	{
		return get_start_location();
	}
	else if ((elapsed_time - hover_time) <= flight_time)
	{
		elapsed_time -= hover_time;
		blas::vector<double> ned_velocity((get_end_location() - get_start_location())/flight_time);
		return get_start_location() + ned_velocity*elapsed_time;
	}
	else
	{
		return get_end_location();
	}
}

double line::get_distance() const
{
	return norm_2(get_end_location() - get_start_location());
}
