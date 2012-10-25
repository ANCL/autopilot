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

#ifndef CIRCLE_H_
#define CIRCLE_H_

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

/**
 * This class defines a circular reference trajectory.  The helicopter
 * is assumed to start on the circle with the nose facing the center.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 25, 2012: Class creation
 */
class circle
{
public:
	circle();
	/// return the reference position for the current time
	blas::vector<double> get_reference_position() const;
	/// reset the trajectory to begin from the current location
	void reset();
protected:
	/// radius of circular trajectory
	double radius;
	///serialize access to radius
	mutable boost::mutex radius_lock;
	/// set the radius
	void set_radius(const double radius) {boost::mutex::scoped_lock lock(radius_lock); this->radius = radius;}
	/// get the radius
	double get_radius() const {boost::mutex::scoped_lock lock(radius_lock); return radius;}

	/// position to begin trajectory in NED frame
	blas::vector<double> start_location;
	/// serialzie access to start_location
	mutable boost::mutex start_location_lock;
	/// set the start_location
	void set_start_location(const blas::vector<double>& start_location) {boost::mutex::scoped_lock(start_location_lock); this->start_location = start_location;}
	/// get the start_location
	blas::vector<double> get_start_location() const {boost::mutex::scoped_lock(start_location_lock); return start_location;}

	/// position of center of circle
	blas::vector<double> center_location;
	/// serialize access to center_location
	mutable boost::mutex center_location_lock;
	/// set the center location based on the current start_location, radius, and heading
	void set_center_location();
	/// get the center location
	blas::vector<double> get_center_location() const {boost::mutex::scoped_lock lock(center_location_lock); return center_location;}

	/// average speed to fly trajectory in m/s
	double speed;
	/// serialize access to speed
	mutable boost::mutex speed_lock;
	/// set the speed
	void set_speed(const double speed) {boost::mutex::scoped_lock(speed_lock); this->speed = speed;}
	/// get the speed
	double get_speed() const {boost::mutex::scoped_lock(speed_lock); return speed;}

	/// time to hover before manouever in seconds
	double hover_time;
	/// serialize access to hover_time
	mutable boost::mutex hover_time_lock;
	/// set the initial hover time before trajectory begins
	void set_hover_time(const double hover_time) {boost::mutex::scoped_lock(hover_time_lock); this->hover_time = hover_time;}
	/// get the hover time
	double get_hover_time() const {boost::mutex::scoped_lock(hover_time_lock); return hover_time;}

	/// time the trajectory started
	boost::posix_time::ptime start_time;
	/// serialize access to start_time
	mutable boost::mutex start_time_lock;
	/// set the start_time to the current time
	void set_start_time() {boost::mutex::scoped_lock lock(start_time_lock); start_time = boost::posix_time::microsec_clock::local_time();}
	/// get the start_Time
	boost::posix_time::ptime get_start_time() const {boost::mutex::scoped_lock lock(start_time_lock); return start_time;}

	/// initial angle (where the helicopter started in polar)
	double initial_angle;
	/// serialize access to initial_angle
	mutable boost::mutex initial_angle_lock;
	/// set the initial angle
	void set_initial_angle(const double initial_angle) {boost::mutex::scoped_lock lock(initial_angle_lock); this->initial_angle = initial_angle;}
	/// get the initial angle
	double get_initial_angle() const {boost::mutex::scoped_lock lock(initial_angle_lock); return initial_angle;}

	/// return circumference of the circular trajectory
	double get_circumference() const;
};

#endif /* CIRCLE_H_ */
