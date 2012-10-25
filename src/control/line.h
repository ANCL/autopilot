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
#ifndef LINE_H_
#define LINE_H_

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

/**
 * Reference line trajectory generator
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 24, 2012: Class creation
 */
class line
{
public:
	line();
	/// return the reference position for the current time
	blas::vector<double> get_reference_position() const;
	/// reset the trajectory to begin from the current location
	void reset();
protected:
	/// position to begin trajectory in NED frame
	blas::vector<double> start_location;
	/// serialzie access to start_location
	mutable boost::mutex start_location_lock;
	/// set the start_location
	void set_start_location(const blas::vector<double>& start_location) {boost::mutex::scoped_lock(start_location_lock); this->start_location = start_location;}
	/// get the start_location
	blas::vector<double> get_start_location() const {boost::mutex::scoped_lock(start_location_lock); return start_location;}

	/// end location in NED frame
	blas::vector<double> end_location;
	/// serialize access to end_location
	mutable boost::mutex end_location_lock;
	/// set the end_location
	void set_end_location(const blas::vector<double>& end_location) {boost::mutex::scoped_lock(end_location_lock); this->end_location = end_location;}
	/// get the end_location
	blas::vector<double> get_end_location() const {boost::mutex::scoped_lock(end_location_lock); return end_location;}

	/// distance to travel in body x direction in m
	double x_travel;
	/// serialize access to x_travel
	mutable boost::mutex x_travel_lock;
	/// set the x_travel
	void set_x_travel(const double x_travel) {boost::mutex::scoped_lock(x_travel_lock); this->x_travel = x_travel;}
	/// get the x_travel
	double get_x_travel() const {boost::mutex::scoped_lock(x_travel_lock); return x_travel;}

	/// distance to travel in body y direction in m
	double y_travel;
	/// serialize access to y_travel
	mutable boost::mutex y_travel_lock;
	/// set y_travel
	void set_y_travel(const double y_travel) {boost::mutex::scoped_lock(y_travel_lock); this->y_travel = y_travel;}
	/// get y_travel
	double get_y_travel() const {boost::mutex::scoped_lock(y_travel_lock); return y_travel;}

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
	/// return trajectory length
	double get_distance() const;
};

#endif /* LINE_H_ */
