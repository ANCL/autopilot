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

/* STL Headers */
#include <string>
#include <vector>

/* Project Headers */
#include "Debug.h"
#include "Parameter.h"

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

	/// set the x_travel
	void set_x_travel(const double x_travel) {{boost::mutex::scoped_lock(x_travel_lock); this->x_travel = x_travel;} message() << "Line: x travel set to " << x_travel;}
	/// get the x_travel
	double get_x_travel() const {boost::mutex::scoped_lock(x_travel_lock); return x_travel;}

	/// set y_travel
	void set_y_travel(const double y_travel) {{boost::mutex::scoped_lock(y_travel_lock); this->y_travel = y_travel;} message() << "Line: y travel set to " << y_travel;}
	/// get y_travel
	double get_y_travel() const {boost::mutex::scoped_lock(y_travel_lock); return y_travel;}

	/// set the initial hover time before trajectory begins
	void set_hover_time(const double hover_time) {{boost::mutex::scoped_lock(hover_time_lock); this->hover_time = hover_time;} message() << "Line: hover time set to " << hover_time;}
	/// get the hover time
	double get_hover_time() const {boost::mutex::scoped_lock(hover_time_lock); return hover_time;}

	/// set the speed
	void set_speed(const double speed) {{boost::mutex::scoped_lock(speed_lock); this->speed = speed;} message() << "Line: speed set to " << speed;}
	/// get the speed
	double get_speed() const {boost::mutex::scoped_lock(speed_lock); return speed;}

	/// return the parameter list to send to qgc
	std::vector<Parameter> getParameters() const;

	/// create and xml tree with the controller parameters
	rapidxml::xml_node<>* get_xml_node(rapidxml::xml_document<>& doc);
	/// parse an xml tree containing the parameters for the function and populate the values
	void parse_xml_node(rapidxml::xml_node<> *line_params);

	static const std::string PARAM_X_TRAVEL;
	static const std::string PARAM_Y_TRAVEL;
	static const std::string PARAM_HOVER_TIME;
	static const std::string PARAM_SPEED;

//protected:
	/// position to begin trajectory in NED frame
	blas::vector<double> start_location;
	/// serialzie access to start_location
	mutable boost::mutex start_location_lock;
	/// set the start_location
	void set_start_location(const blas::vector<double>& start_location) {{boost::mutex::scoped_lock(start_location_lock); this->start_location = start_location;} message() << "Line: Start Location set to " << start_location;}
	/// get the start_location
	blas::vector<double> get_start_location() const {boost::mutex::scoped_lock(start_location_lock); return start_location;}

	/// end location in NED frame
	blas::vector<double> end_location;
	/// serialize access to end_location
	mutable boost::mutex end_location_lock;
	/// set the end_location
	void set_end_location(const blas::vector<double>& end_location) {{boost::mutex::scoped_lock(end_location_lock); this->end_location = end_location;} message() << "Line: End location set to " << end_location;}
	/// get the end_location
	blas::vector<double> get_end_location() const {boost::mutex::scoped_lock(end_location_lock); return end_location;}

	/// distance to travel in body x direction in m
	double x_travel;
	/// serialize access to x_travel
	mutable boost::mutex x_travel_lock;

	/// distance to travel in body y direction in m
	double y_travel;
	/// serialize access to y_travel
	mutable boost::mutex y_travel_lock;

	/// average speed to fly trajectory in m/s
	double speed;
	/// serialize access to speed
	mutable boost::mutex speed_lock;

	/// time to hover before manouever in seconds
	double hover_time;
	/// serialize access to hover_time
	mutable boost::mutex hover_time_lock;

	/// time the trajectory started
	boost::posix_time::ptime start_time;
	/// serialize access to start_time
	mutable boost::mutex start_time_lock;
	/// set the start_time to the current time
	void set_start_time() {{boost::mutex::scoped_lock lock(start_time_lock); start_time = boost::posix_time::microsec_clock::local_time();} message() << "Line trajectory started";}
	/// get the start_Time
	boost::posix_time::ptime get_start_time() const {boost::mutex::scoped_lock lock(start_time_lock); return start_time;}

	/// return trajectory length
	double get_distance() const;
};

#endif /* LINE_H_ */
