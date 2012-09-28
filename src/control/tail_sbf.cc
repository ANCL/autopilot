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

/* Project Headers */
#include "IMU.h"
#include "tail_sbf.h"
#include "Helicopter.h"
#include "Control.h"

/* STL Headers */
#include <math.h>

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

	blas::vector<double> ned_control(3);
	ned_control.clear();
	std::vector<double> error_states;
	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		error_states.push_back(ned_x.error().proportional() = ned_position_error(0));
		error_states.push_back(ned_x.error().derivative() = ned_velocity_error(0));
		error_states.push_back(++(ned_x.error()));
		ned_control(0) = ned_x.compute_pid();
	}
	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		error_states.push_back(ned_y.error().proportional() = ned_position_error(1));
		error_states.push_back(ned_y.error().derivative() = ned_velocity_error(1));
		error_states.push_back(++(ned_y.error()));
		ned_control(1) = ned_y.compute_pid();
	}

	LogFile::getInstance()->logData(heli::LOG_TRANS_SBF_ERROR_STATES, error_states);

	double heading = imu->get_euler()(2);
	blas::matrix<double> Rz(3,3);
	Rz.clear();
	Rz(0,0) = cos(heading);
	Rz(0,1) = -sin(heading);
	Rz(1,0) = sin(heading);
	Rz(1,1) = cos(heading);
	Rz(2,2) = 1;

	Helicopter* bergen = Helicopter::getInstance();
	double m = bergen->get_mass();
	double g = bergen->get_gravity();
	ned_control(2) = -g;
	blas::vector<double> body_control(m*prod(trans(Rz),ned_control));
	double alpham = 0.04; // countertorque approximate slope
	double xt = abs(bergen->get_tail_hub_offset()(0));

	double theta_ref = atan(body_control(0)/body_control(2));
	double phi_ref = -atan((alpham*sqrt(pow(body_control(0),2)+pow(body_control(2),2))+xt*body_control(1))/(alpham*body_control(1) - xt*sqrt(pow(body_control(0),2)+pow(body_control(2),2))));

	blas::vector<double> attitude_reference(2);
	attitude_reference(0) = phi_ref;
	attitude_reference(1) = theta_ref;

	Control::saturate(attitude_reference, scaled_travel_radians());

	set_control_effort(attitude_reference);
}

const std::string tail_sbf::PARAM_X_KP = "SBF_X_KP";
const std::string tail_sbf::PARAM_X_KD = "SBF_X_KD";
const std::string tail_sbf::PARAM_X_KI = "SBF_X_KI";

const std::string tail_sbf::PARAM_Y_KP = "SBF_Y_KP";
const std::string tail_sbf::PARAM_Y_KD = "SBF_Y_KD";
const std::string tail_sbf::PARAM_Y_KI = "SBF_Y_KI";

const std::string tail_sbf::PARAM_TRAVEL = "SBF_TRAVEL";

std::vector<Parameter> tail_sbf::getParameters() const
{
	std::vector<Parameter> plist;

	{
		boost::mutex::scoped_lock lock(ned_x_lock);
		plist.push_back(Parameter(PARAM_X_KP, ned_x.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KD, ned_x.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KI, ned_x.gains().integral(), heli::CONTROLLER_ID));
	}

	{
		boost::mutex::scoped_lock lock(ned_y_lock);
		plist.push_back(Parameter(PARAM_Y_KP, ned_y.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KD, ned_y.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KI, ned_y.gains().integral(), heli::CONTROLLER_ID));
	}

	plist.push_back(Parameter(PARAM_TRAVEL, scaled_travel_degrees(), heli::CONTROLLER_ID));

	return plist;
}
