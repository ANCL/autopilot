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
	blas::vector<double> get_reference_position();
	/// reset the trajectory to begin from the current location
	void reset();
protected:
	/// position to begin trajectory in NED frame
	blas::vector<double> start_location;
	/// distance to travel in x direction in m
	double x_travel;
	/// distance to travel in y direction in m
	double y_travel;
	/// average speed to fly trajectory in m/s
	double speed;
	/// time to hover before manouever in seconds
	double hover_time;
	/// time to start the trajectory
	boost::posix_time::ptime start_time;
};

#endif /* LINE_H_ */
