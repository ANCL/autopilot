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

#include "tail_sbf.h"
#include "IMU.h"

tail_sbf::tail_sbf() {


}

void tail_sbf::reset()
{
	ned_x.reset();
	ned_y.reset();
}

bool tail_sbf::runnable() const
{
	return true;
}

void tail_sbf::operator()(const blas::vector<double>& reference) throw(bad_control)
{

	IMU* imu = IMU::getInstance();
	blas::vector<double> ned_position_error(imu->get_ned_position() - reference);
	blas::vector<double> ned_velocity_error(imu->get_ned_velocity());

//	blas::vector<double>
//	{
//		boost::mutex::scoped_lock lock(x_lock);
//		x.error().proportional() = body_position_error[0];
//		x.error().derivative() = body_velocity_error[0];
//		++(x.error());
//		attitude_reference[1] = x.compute_pid();
//	}
//	{
//		boost::mutex::scoped_lock lock(y_lock);
//		y.error().proportional() = body_position_error[1];
//		y.error().derivative() = body_velocity_error[1];
//		++(y.error());
//		attitude_reference[0] = -y.compute_pid();
//	}
}

blas::vector<double> tail_sbf::get_control_effort() const
{

}
